/*
 * Copyright 2013 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 */

#include <asm/homecache.h>

#include <linux/crypto.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/rtnetlink.h>

#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/des.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/scatterwalk.h>

#include <gxio/gxcr.h>
#include <gxio/aead.h>
#include <gxio/token_aead.h>
#include <gxio/iorpc_mica.h>

static int tile_num_hw_contexts = 10;
module_param(tile_num_hw_contexts, int, 0444);
MODULE_PARM_DESC(tile_num_hw_contexts,
		 "Number of MiCA hardware contexts to allocate");
MODULE_AUTHOR("Tilera Corporation");
MODULE_LICENSE("GPL");

#define JUMBO_PACKET_BUF_SIZE 10240

struct tile_crypto_alg {
	struct crypto_alg alg;
	gxcr_cipher_t cipher;
	gxcr_digest_t digest;
};

struct tile_hw_ctx {
	gxio_mica_context_t mica_ctx;
	unsigned char packet_buffer[JUMBO_PACKET_BUF_SIZE];
};

static struct tile_crypto_alg tile_algs[] = {
	{
		.alg = {
			.cra_name	= "authenc(hmac(sha1),cbc(des))",
			.cra_blocksize	= DES_BLOCK_SIZE,
			.cra_u		= { .aead = {
					.ivsize		= DES_BLOCK_SIZE,
					.maxauthsize	= SHA1_DIGEST_SIZE,
				}
			}
		},
		.cipher = GXCR_CIPHER_DES_CBC,
		.digest = GXCR_DIGEST_SHA1,
	}, {
		.alg = {
			.cra_name	= "authenc(hmac(md5),cbc(aes))",
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_u		= { .aead = {
					.ivsize		= AES_BLOCK_SIZE,
					.maxauthsize	= MD5_DIGEST_SIZE,
				}
			}
		},
		.cipher = GXCR_CIPHER_AES_CBC_128,
		.digest = GXCR_DIGEST_MD5,
	}, {
		.alg	= {
			.cra_name	= "authenc(hmac(sha1),cbc(aes))",
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_u		= { .aead = {
					.ivsize		= AES_BLOCK_SIZE,
					.maxauthsize	= SHA1_DIGEST_SIZE,
				}
			}
		},
		.cipher = GXCR_CIPHER_AES_CBC_128,
		.digest = GXCR_DIGEST_SHA1,
	} ,
{
		.alg	= {
			.cra_name	= "authenc(hmac(sha256),cbc(aes))",
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_u		= { .aead = {
					.ivsize		= AES_BLOCK_SIZE,
					.maxauthsize	= SHA256_DIGEST_SIZE,
				}
			}
		},
		.cipher = GXCR_CIPHER_AES_CBC_128,
		.digest = GXCR_DIGEST_SHA_256,
}
};

static struct tile_hw_ctx *alloc_mica_context(int shim)
{
	pte_t hash_pte = pte_set_home((pte_t) { 0 }, PAGE_HOME_HASH);
	struct tile_hw_ctx *pcb = kmalloc(sizeof(struct tile_hw_ctx),
					  GFP_KERNEL);
	int res;
	if (!pcb)
		return 0;

	res = gxio_mica_init(&pcb->mica_ctx, GXIO_MICA_ACCEL_CRYPTO, shim);
	if (res) {
		kfree(pcb);
		return 0;
	}

	res = __gxio_mica_register_client_memory(pcb->mica_ctx.fd, 0,
						 hash_pte, 0);
	if (res) {
		kfree(pcb);
		gxio_mica_destroy(&pcb->mica_ctx);
		return 0;
	}

	return pcb;
}

/* FIXME: still need to decide how to make use of both shims.  Maybe
 * configuration time param to decide which one shim, or both shims, and
 * if both shims then put them into pool interleaved, figuring they'd get
 * about equal use?  One pool per shim.
 */
static struct tile_hw_ctx **mica_ctx_pool;
static spinlock_t ctx_pool_lock;

static int init_mica_context_pool(int shim)
{
	int i;
	unsigned long flags;
	int result = 0;

	spin_lock_irqsave(&ctx_pool_lock, flags);

	mica_ctx_pool =
		kmalloc(sizeof(struct tile_hw_ctx *) * tile_num_hw_contexts,
			GFP_KERNEL);
	if (!mica_ctx_pool) {
		spin_unlock_irqrestore(&ctx_pool_lock, flags);
		return -ENOMEM;
	}

	for (i = 0; i < tile_num_hw_contexts; i++) {
		mica_ctx_pool[i] = alloc_mica_context(shim);
		if (mica_ctx_pool[i] == 0) {
			printk(KERN_WARNING "crypto driver allocating %d "
			       "contexts on shim %d, requested %d\n",
			       i, shim, tile_num_hw_contexts);
			tile_num_hw_contexts = i;

			if (i == 0) {
				result = -EBUSY;
				kfree(mica_ctx_pool);
				mica_ctx_pool = NULL;
				break;
			}
		}
	}

	spin_unlock_irqrestore(&ctx_pool_lock, flags);

	return result;
}

static struct tile_hw_ctx *get_mica_context(int shim)
{
	int i;
	struct tile_hw_ctx *pcb = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ctx_pool_lock, flags);

	for (i = 0; i < tile_num_hw_contexts; i++)
		if (mica_ctx_pool[i]) {
			pcb = mica_ctx_pool[i];
			mica_ctx_pool[i] = NULL;
			break;
		}

	spin_unlock_irqrestore(&ctx_pool_lock, flags);

	return pcb;
}

static void release_mica_context(struct tile_hw_ctx *pcb, int shim)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&ctx_pool_lock, flags);

	for (i = 0; i < tile_num_hw_contexts; i++) {
		if (mica_ctx_pool[i] == NULL) {
			mica_ctx_pool[i] = pcb;
			break;
		}
		BUG_ON(i == tile_num_hw_contexts);
	}

	spin_unlock_irqrestore(&ctx_pool_lock, flags);
}

static int aead_setkey(struct crypto_aead *tfm, const u8 *key,
			unsigned int keylen)
{
	gxcr_aead_context_t *ctx = crypto_aead_ctx(tfm);
	unsigned int authkeylen;
	unsigned int enckeylen;
	struct crypto_authenc_key_param *param;
	struct rtattr *rta = (void *)key;
	struct tile_hw_ctx *pcb;
	int res;

	if (!RTA_OK(rta, keylen))
		goto badkey;

	if (rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM)
		goto badkey;

	if (RTA_PAYLOAD(rta) < sizeof(*param))
		goto badkey;

	param = RTA_DATA(rta);
	enckeylen = be32_to_cpu(param->enckeylen);

	key += RTA_ALIGN(rta->rta_len);
	keylen -= RTA_ALIGN(rta->rta_len);

	if (keylen < enckeylen)
		goto badkey;

	authkeylen = keylen - enckeylen;

	if (authkeylen <= 0) {
		printk(KERN_WARNING "TILE-Gx non-hmac digests not yet "
		       "supported\n");
		goto badkey;
	}

	memcpy(gxcr_aead_context_key(ctx), key + authkeylen, enckeylen);

	pcb = get_mica_context(0);

	res = gxcr_aead_precalc(&pcb->mica_ctx, ctx,
				pcb->packet_buffer,
				sizeof(pcb->packet_buffer),
				(unsigned char *)key, authkeylen);

	release_mica_context(pcb, 0);

	if (res)
		goto badkey;

	return 0;

badkey:
	crypto_aead_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
	return -EINVAL;
}

static int aead_perform(struct aead_request *req, int encrypt, int geniv,
			unsigned char *iv)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	gxcr_aead_context_t *ctx = crypto_aead_ctx(tfm);
	unsigned int ivsize = crypto_aead_ivsize(tfm);
	unsigned int authsize = crypto_aead_authsize(tfm);
	unsigned int dst_len = encrypt ?
		req->cryptlen + req->assoclen + authsize :
		req->cryptlen + req->assoclen;
	unsigned int copy_len = encrypt ?
		req->cryptlen + authsize : req->cryptlen;
	struct tile_hw_ctx *pcb = get_mica_context(0);
	unsigned char *src_data;
	gxcr_result_token_t *res;
	int result = 0;

	src_data = pcb->packet_buffer;

	scatterwalk_map_and_copy(src_data, req->assoc, 0, req->assoclen, 0);
	if (!geniv) {
		memcpy(src_data + req->assoclen, req->iv, ivsize);
		memcpy(gxcr_aead_context_iv(ctx), req->iv, ivsize);
		scatterwalk_map_and_copy(src_data + req->assoclen + ivsize,
					 req->src, 0, req->cryptlen, 0);
	} else
		scatterwalk_map_and_copy(src_data + req->assoclen, req->src,
					 0, req->cryptlen, 0);

	aead_process_packet(&pcb->mica_ctx, ctx, src_data,
			    req->cryptlen + req->assoclen + ivsize,
			    req->assoclen + ivsize, src_data,
			    dst_len, authsize, encrypt, geniv, ivsize);

	if (geniv && iv)
		memcpy(iv, src_data + req->assoclen, ivsize);

	scatterwalk_map_and_copy(src_data + req->assoclen + ivsize, req->dst,
				 0, copy_len, 1);

	res = gxcr_aead_result(ctx);

	if (res->e0_e14)
		result = -EBADMSG;

	release_mica_context(pcb, 0);

	return result;
}

static int aead_encrypt(struct aead_request *req)
{
	return aead_perform(req, 1, 0, NULL);
}

static int aead_decrypt(struct aead_request *req)
{
	return aead_perform(req, 0, 0, NULL);
}

static int aead_givencrypt(struct aead_givcrypt_request *req)
{
	aead_perform(&req->areq, 1, 1, req->giv);
	return 0;
}

static int init_tfm_aead(struct crypto_tfm *tfm)
{
	gxcr_aead_context_t *ctx = crypto_tfm_ctx(tfm);
	struct tile_crypto_alg *cra = container_of(tfm->__crt_alg,
						   struct tile_crypto_alg,
						   alg);
	gxcr_cipher_t cipher = cra->cipher;
	gxcr_digest_t digest = cra->digest;
	gxcr_aead_params_t params = {
		.token_template = &aead_token_info,
		.cipher = cipher,
		.digest = digest,
		.hmac_mode = 1,
	};
	unsigned char *metadata_mem;
	int metadata_mem_size;
	int result;

	metadata_mem_size = gxcr_aead_calc_context_bytes(&params);
	metadata_mem = kmalloc(metadata_mem_size, GFP_KERNEL);
	if (!metadata_mem)
		return -ENOMEM;

	result = gxcr_aead_init_context(ctx, metadata_mem,
					metadata_mem_size,
					&params, NULL, NULL);
	if (result) {
		kfree(metadata_mem);
		return result;
	}

	aead_setup(ctx);

	return result;
}

static void exit_tfm(struct crypto_tfm *tfm)
{
	gxcr_aead_context_t *ctx = crypto_tfm_ctx(tfm);
	kfree(ctx->metadata_mem);
	ctx->metadata_mem = NULL;
}

static int tile_remove(struct platform_device *ofdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tile_algs); i++)
		crypto_unregister_alg(&tile_algs[i].alg);

	return 0;
}

static int tile_probe(struct platform_device *ofdev)
{
	int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(tile_algs); i++) {
		struct crypto_alg *cra = &tile_algs[i].alg;
		cra->cra_type = &crypto_aead_type;
		cra->cra_flags = CRYPTO_ALG_TYPE_AEAD |
			CRYPTO_ALG_ASYNC;
		cra->cra_aead.setkey = aead_setkey;
		cra->cra_aead.encrypt = aead_encrypt;
		cra->cra_aead.decrypt = aead_decrypt;
		cra->cra_aead.givencrypt = aead_givencrypt;
		cra->cra_init = init_tfm_aead;
		cra->cra_ctxsize = sizeof(gxcr_aead_context_t);
		cra->cra_module = THIS_MODULE;
		cra->cra_alignmask = 0;
		cra->cra_priority = 300;
		cra->cra_exit = exit_tfm;
		err = crypto_register_alg(cra);
		if (err) {
			printk(KERN_ERR "Failed to register TILE-Gx '%s'\n",
			       cra->cra_name);
			goto err_out;
		}
	}

	return 0;

err_out:
	tile_remove(ofdev);

	return err;
}

static struct platform_driver tile_driver = {
	.driver = {
		.name = "tile",
		.owner = THIS_MODULE,
	},
	.probe = tile_probe,
	.remove = tile_remove,
};

/* Module initialization. */
static int __init tile_module_init(void)
{
	int err;

	spin_lock_init(&ctx_pool_lock);

	err = init_mica_context_pool(0);
	if (err)
		return err;

	return platform_driver_register(&tile_driver);
}

static void __exit tile_module_exit(void)
{
	platform_driver_unregister(&tile_driver);
}

module_init(tile_module_init);
module_exit(tile_module_exit);
