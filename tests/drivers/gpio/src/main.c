#include <zephyr/ztest.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

ZTEST(gpio_tt_suite, test_gpio)
{
    zassert_true(1, "GPIO test passed");

    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    zassert_true(device_is_ready(dev));
}

ZTEST_SUITE(gpio_tt_suite, NULL, NULL, NULL, NULL, NULL);
