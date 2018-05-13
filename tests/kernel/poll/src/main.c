/*
 * Copyright (c) 2017 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
extern void test_poll_no_wait(void);
extern void test_poll_wait(void);
extern void test_poll_multi(void);

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(poll_api,
			ztest_unit_test(test_poll_no_wait),
			ztest_unit_test(test_poll_wait),
			ztest_unit_test(test_poll_multi));
	ztest_run_test_suite(poll_api);
}
