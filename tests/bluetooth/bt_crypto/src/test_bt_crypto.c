/* Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest_assert.h>

#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>
#include "bt_crypto.h"

ZTEST_SUITE(bt_crypto, NULL, NULL, NULL, NULL, NULL);

ZTEST(bt_crypto, test_result_aes_cmac)
{
	static const uint8_t key[] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
				      0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
	static const uint8_t M[] = {
		0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73,
		0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7,
		0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4,
		0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45,
		0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};

	uint8_t exp_mac1[] = {0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28,
			      0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75, 0x67, 0x46};
	uint8_t exp_mac2[] = {0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
			      0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c};
	uint8_t exp_mac3[] = {0xdf, 0xa6, 0x67, 0x47, 0xde, 0x9a, 0xe6, 0x30,
			      0x30, 0xca, 0x32, 0x61, 0x14, 0x97, 0xc8, 0x27};
	uint8_t exp_mac4[] = {0x51, 0xf0, 0xbe, 0xbf, 0x7e, 0x3b, 0x9d, 0x92,
			      0xfc, 0x49, 0x74, 0x17, 0x79, 0x36, 0x3c, 0xfe};
	uint8_t res[16];

	bt_crypto_aes_cmac(key, M, 0, res);
	zassert_mem_equal(res, exp_mac1, 16);

	bt_crypto_aes_cmac(key, M, 16, res);
	zassert_mem_equal(res, exp_mac2, 16);

	bt_crypto_aes_cmac(key, M, 40, res);
	zassert_mem_equal(res, exp_mac3, 16);

	bt_crypto_aes_cmac(key, M, 64, res);
	zassert_mem_equal(res, exp_mac4, 16);
}

ZTEST(bt_crypto, test_result_f4)
{
	uint8_t u[32] = {0xe6, 0x9d, 0x35, 0x0e, 0x48, 0x01, 0x03, 0xcc, 0xdb, 0xfd, 0xf4,
			 0xac, 0x11, 0x91, 0xf4, 0xef, 0xb9, 0xa5, 0xf9, 0xe9, 0xa7, 0x83,
			 0x2c, 0x5e, 0x2c, 0xbe, 0x97, 0xf2, 0xd2, 0x03, 0xb0, 0x20};
	uint8_t v[32] = {0xfd, 0xc5, 0x7f, 0xf4, 0x49, 0xdd, 0x4f, 0x6b, 0xfb, 0x7c, 0x9d,
			 0xf1, 0xc2, 0x9a, 0xcb, 0x59, 0x2a, 0xe7, 0xd4, 0xee, 0xfb, 0xfc,
			 0x0a, 0x90, 0x9a, 0xbb, 0xf6, 0x32, 0x3d, 0x8b, 0x18, 0x55};
	uint8_t x[16] = {0xab, 0xae, 0x2b, 0x71, 0xec, 0xb2, 0xff, 0xff,
			 0x3e, 0x73, 0x77, 0xd1, 0x54, 0x84, 0xcb, 0xd5};
	uint8_t z = 0x00;

	uint8_t exp_res[16] = {0x2d, 0x87, 0x74, 0xa9, 0xbe, 0xa1, 0xed, 0xf1,
			       0x1c, 0xbd, 0xa9, 0x07, 0xf1, 0x16, 0xc9, 0xf2};
	uint8_t res[16];

	bt_crypto_f4(u, v, x, z, res);
	zassert_mem_equal(res, exp_res, 16);
}

ZTEST(bt_crypto, test_result_f5)
{
	uint8_t w[32] = {0x98, 0xa6, 0xbf, 0x73, 0xf3, 0x34, 0x8d, 0x86, 0xf1, 0x66, 0xf8,
			 0xb4, 0x13, 0x6b, 0x79, 0x99, 0x9b, 0x7d, 0x39, 0x0a, 0xa6, 0x10,
			 0x10, 0x34, 0x05, 0xad, 0xc8, 0x57, 0xa3, 0x34, 0x02, 0xec};
	uint8_t n1[16] = {0xab, 0xae, 0x2b, 0x71, 0xec, 0xb2, 0xff, 0xff,
			  0x3e, 0x73, 0x77, 0xd1, 0x54, 0x84, 0xcb, 0xd5};
	uint8_t n2[16] = {0xcf, 0xc4, 0x3d, 0xff, 0xf7, 0x83, 0x65, 0x21,
			  0x6e, 0x5f, 0xa7, 0x25, 0xcc, 0xe7, 0xe8, 0xa6};
	bt_addr_le_t a1 = {.type = 0x00, .a.val = {0xce, 0xbf, 0x37, 0x37, 0x12, 0x56}};
	bt_addr_le_t a2 = {.type = 0x00, .a.val = {0xc1, 0xcf, 0x2d, 0x70, 0x13, 0xa7}};

	uint8_t exp_ltk[16] = {0x38, 0x0a, 0x75, 0x94, 0xb5, 0x22, 0x05, 0x98,
			       0x23, 0xcd, 0xd7, 0x69, 0x11, 0x79, 0x86, 0x69};
	uint8_t exp_mackey[16] = {0x20, 0x6e, 0x63, 0xce, 0x20, 0x6a, 0x3f, 0xfd,
				  0x02, 0x4a, 0x08, 0xa1, 0x76, 0xf1, 0x65, 0x29};
	uint8_t mackey[16], ltk[16];

	bt_crypto_f5(w, n1, n2, &a1, &a2, mackey, ltk);
	zassert_mem_equal(mackey, exp_mackey, 16);
	zassert_mem_equal(ltk, exp_ltk, 16);
}

ZTEST(bt_crypto, test_result_f6)
{
	uint8_t w[16] = {0x20, 0x6e, 0x63, 0xce, 0x20, 0x6a, 0x3f, 0xfd,
			 0x02, 0x4a, 0x08, 0xa1, 0x76, 0xf1, 0x65, 0x29};
	uint8_t n1[16] = {0xab, 0xae, 0x2b, 0x71, 0xec, 0xb2, 0xff, 0xff,
			  0x3e, 0x73, 0x77, 0xd1, 0x54, 0x84, 0xcb, 0xd5};
	uint8_t n2[16] = {0xcf, 0xc4, 0x3d, 0xff, 0xf7, 0x83, 0x65, 0x21,
			  0x6e, 0x5f, 0xa7, 0x25, 0xcc, 0xe7, 0xe8, 0xa6};
	uint8_t r[16] = {0xc8, 0x0f, 0x2d, 0x0c, 0xd2, 0x42, 0xda, 0x08,
			 0x54, 0xbb, 0x53, 0xb4, 0x3b, 0x34, 0xa3, 0x12};
	uint8_t io_cap[3] = {0x02, 0x01, 0x01};
	bt_addr_le_t a1 = {.type = 0x00, .a.val = {0xce, 0xbf, 0x37, 0x37, 0x12, 0x56}};
	bt_addr_le_t a2 = {.type = 0x00, .a.val = {0xc1, 0xcf, 0x2d, 0x70, 0x13, 0xa7}};

	uint8_t exp_res[16] = {0x61, 0x8f, 0x95, 0xda, 0x09, 0x0b, 0x6c, 0xd2,
			       0xc5, 0xe8, 0xd0, 0x9c, 0x98, 0x73, 0xc4, 0xe3};
	uint8_t res[16];

	bt_crypto_f6(w, n1, n2, r, io_cap, &a1, &a2, res);
	zassert_mem_equal(res, exp_res, 16);
}

ZTEST(bt_crypto, test_result_g2)
{
	uint8_t u[32] = {0xe6, 0x9d, 0x35, 0x0e, 0x48, 0x01, 0x03, 0xcc, 0xdb, 0xfd, 0xf4,
			 0xac, 0x11, 0x91, 0xf4, 0xef, 0xb9, 0xa5, 0xf9, 0xe9, 0xa7, 0x83,
			 0x2c, 0x5e, 0x2c, 0xbe, 0x97, 0xf2, 0xd2, 0x03, 0xb0, 0x20};
	uint8_t v[32] = {0xfd, 0xc5, 0x7f, 0xf4, 0x49, 0xdd, 0x4f, 0x6b, 0xfb, 0x7c, 0x9d,
			 0xf1, 0xc2, 0x9a, 0xcb, 0x59, 0x2a, 0xe7, 0xd4, 0xee, 0xfb, 0xfc,
			 0x0a, 0x90, 0x9a, 0xbb, 0xf6, 0x32, 0x3d, 0x8b, 0x18, 0x55};
	uint8_t x[16] = {0xab, 0xae, 0x2b, 0x71, 0xec, 0xb2, 0xff, 0xff,
			 0x3e, 0x73, 0x77, 0xd1, 0x54, 0x84, 0xcb, 0xd5};
	uint8_t y[16] = {0xcf, 0xc4, 0x3d, 0xff, 0xf7, 0x83, 0x65, 0x21,
			 0x6e, 0x5f, 0xa7, 0x25, 0xcc, 0xe7, 0xe8, 0xa6};

	uint32_t exp_res = 0x2f9ed5ba % 1000000;
	uint32_t res;

	bt_crypto_g2(u, v, x, y, &res);
	zassert_equal(res, exp_res);
}

ZTEST(bt_crypto, test_result_h6)
{
	uint8_t w[16] = {0x9b, 0x7d, 0x39, 0x0a, 0xa6, 0x10, 0x10, 0x34,
			 0x05, 0xad, 0xc8, 0x57, 0xa3, 0x34, 0x02, 0xec};
	uint8_t key_id[4] = {0x72, 0x62, 0x65, 0x6c};

	uint8_t exp_res[16] = {0x99, 0x63, 0xb1, 0x80, 0xe2, 0xa9, 0xd3, 0xe8,
			       0x1c, 0xc9, 0x6d, 0xe7, 0x02, 0xe1, 0x9a, 0x2d};
	uint8_t res[16];

	bt_crypto_h6(w, key_id, res);
	zassert_mem_equal(res, exp_res, 16);
}

ZTEST(bt_crypto, test_result_h7)
{
	uint8_t salt[16] = {0x31, 0x70, 0x6d, 0x74, 0x00, 0x00, 0x00, 0x00,
			    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t w[16] = {0x9b, 0x7d, 0x39, 0x0a, 0xa6, 0x10, 0x10, 0x34,
			 0x05, 0xad, 0xc8, 0x57, 0xa3, 0x34, 0x02, 0xec};

	uint8_t exp_res[16] = {0x11, 0x70, 0xa5, 0x75, 0x2a, 0x8c, 0x99, 0xd2,
			       0xec, 0xc0, 0xa3, 0xc6, 0x97, 0x35, 0x17, 0xfb};
	uint8_t res[16];

	bt_crypto_h7(salt, w, res);
	zassert_mem_equal(res, exp_res, 16);
}

ZTEST(bt_crypto, test_result_h8)
{
	uint8_t k[16] = {0x9b, 0x7d, 0x39, 0x0a, 0xa6, 0x10, 0x10, 0x34,
			 0x05, 0xad, 0xc8, 0x57, 0xa3, 0x34, 0x02, 0xec};
	uint8_t s[16] = {0xba, 0xd5, 0x9e, 0x2f, 0xc1, 0x44, 0x70, 0x9b,
			 0xf9, 0x0d, 0xd2, 0xe3, 0x8d, 0xd1, 0x36, 0x15};
	uint8_t key_id[4] = {0x48, 0x01, 0x03, 0xcc};

	uint8_t exp_res[16] = {0x6d, 0x0f, 0x35, 0xed, 0x04, 0x89, 0xa3, 0x22,
			       0xe7, 0x28, 0x72, 0xae, 0xba, 0xbe, 0xe5, 0xe5};
	uint8_t res[16];

	bt_crypto_h8(k, s, key_id, res);
	zassert_mem_equal(res, exp_res, 16);
}
