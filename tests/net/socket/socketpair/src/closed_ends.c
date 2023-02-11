/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 */

#include "_main.h"

ZTEST_USER(net_socketpair, close_one_end_and_write_to_the_other)
{
	int res;
	int sv[2] = {-1, -1};

	for (size_t i = 0; i < 2; ++i) {
		res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
		zassert_equal(res, 0, "socketpair(2) failed: %d", errno);

		res = close(sv[i]);
		zassert_equal(res, 0, "close(sv[%u]) failed: %d", i, errno);

		res = write(sv[(!i) & 1], "x", 1);
		zassert_equal(res, -1, "expected write(2) to fail");
		zassert_equal(res, -1, "errno: expected: EPIPE actual: %d",
			errno);

		res = close(sv[(!i) & 1]);
		zassert_equal(res, 0, "close(sv[%u]) failed: %d", i, errno);
	}
}

ZTEST_USER(net_socketpair, close_one_end_and_read_from_the_other)
{
	int res;
	int sv[2] = {-1, -1};
	char xx[16];

	for (size_t i = 0; i < 2; ++i) {
		res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
		zassert_equal(res, 0, "socketpair(2) failed: %d", errno);

		/* We want to write some bytes to the closing end of the
		 * socket before it gets shut down, so that we can prove that
		 * reading is possible from the other end still and that data
		 * is not lost.
		 */
		res = write(sv[i], "xx", 2);
		zassert_not_equal(res, -1, "write(2) failed: %d", errno);
		zassert_equal(res, 2, "write(2) failed to write 2 bytes");

		res = close(sv[i]);
		zassert_equal(res, 0, "close(sv[%u]) failed: %d", i, errno);

		memset(xx, 0, sizeof(xx));
		res = read(sv[(!i) & 1], xx, sizeof(xx));
		zassert_not_equal(res, -1, "read(2) failed: %d", errno);
		zassert_equal(res, 2, "expected to read 2 bytes but read %d",
			res);

		res = read(sv[(!i) & 1], xx, sizeof(xx));
		zassert_equal(res, 0,
			"expected read(2) to succeed but read 0 bytes");

		res = close(sv[(!i) & 1]);
		zassert_equal(res, 0, "close(sv[%u]) failed: %d", i, errno);
	}
}
