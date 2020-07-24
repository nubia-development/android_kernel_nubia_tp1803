/*
 * Hisilicon thermal sensor driver
 *
 * Copyright (c) 2014-2015 Hisilicon Limited.
 * Copyright (c) 2014-2015 Linaro Limited.
 *
 * Xinwei Kong <kong.kongxinwei@hisilicon.com>
 * Leo Yan <leo.yan@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "thermal_core.h"


struct hisi_thermal_sensor {
	struct hisi_thermal_data *thermal;
	struct thermal_zone_device *tzd;

	long sensor_temp;
	uint32_t id;
	uint32_t thres_temp;
};

struct hisi_thermal_data {
	struct mutex thermal_lock;    /* protects register data */
	struct platform_device *pdev;
	struct clk *clk;
	void __iomem *regs;
};

/*
 * The temperature computation on the tsensor is as follow:
 *	Unit: millidegree Celsius
 *	Step: 255/200 (0.7843)
 *	Temperature base: -60°C
 *
 * The register is programmed in temperature steps, every step is 784
 * millidegree and begins at -60 000 m°C
 *
 * The temperature from the steps:
 *
 *	Temp = TempBase + (steps x 784)
 *
 * and the steps from the temperature:
 *
 *	steps = (Temp - TempBase) / 784
 *
 */
static inline int hisi_thermal_step_to_temp(int step)
{
	return HISI_TEMP_BASE + (step * HISI_TEMP_STEP);
}

static inline long hisi_thermal_temp_to_step(long temp)
{
	return (temp - HISI_TEMP_BASE) / HISI_TEMP_STEP;
}

static inline long hisi_thermal_round_temp(int temp)
{
	return hisi_thermal_step_to_temp(
		hisi_thermal_temp_to_step(temp));
}

}

static void hisi_thermal_disable_sensor(struct hisi_thermal_data *data)
{
	mutex_lock(&data->thermal_lock);

	mutex_unlock(&data->thermal_lock);
}

static int hisi_thermal_get_temp(void *_sensor, int *temp)
{
	struct hisi_thermal_sensor *sensor = _sensor;
	struct hisi_thermal_data *data = sensor->thermal;


	dev_dbg(&data->pdev->dev, "id=%d, temp=%d, thres=%d\n",
		sensor->id, *temp, sensor->thres_temp);

	return 0;
}

static const struct thermal_zone_of_device_ops hisi_of_thermal_ops = {
	.get_temp = hisi_thermal_get_temp,
};

static irqreturn_t hisi_thermal_alarm_irq_thread(int irq, void *dev)
{
	struct hisi_thermal_data *data = dev;

	if (temp >= sensor->thres_temp) {
		dev_crit(&data->pdev->dev, "THERMAL ALARM: %d > %d\n",
			 temp, sensor->thres_temp);

		dev_crit(&data->pdev->dev, "THERMAL ALARM stopped: %d < %d\n",
			 temp, sensor->thres_temp);
	}

	return IRQ_HANDLED;
}

static int hisi_thermal_register_sensor(struct platform_device *pdev,
					struct hisi_thermal_data *data,
					struct hisi_thermal_sensor *sensor,
					int index)
{
	int ret, i;
	const struct thermal_trip *trip;

	sensor->id = index;
	sensor->thermal = data;

	sensor->tzd = devm_thermal_zone_of_sensor_register(&pdev->dev,
				sensor->id, sensor, &hisi_of_thermal_ops);
	if (IS_ERR(sensor->tzd)) {
		ret = PTR_ERR(sensor->tzd);
		sensor->tzd = NULL;
		dev_err(&pdev->dev, "failed to register sensor id %d: %d\n",
			sensor->id, ret);
		return ret;
	}

	trip = of_thermal_get_trip_points(sensor->tzd);

	for (i = 0; i < of_thermal_get_ntrips(sensor->tzd); i++) {
		if (trip[i].type == THERMAL_TRIP_PASSIVE) {
			sensor->thres_temp = hisi_thermal_round_temp(trip[i].temperature);
			break;
		}
	}

	return 0;
}

static const struct of_device_id of_hisi_thermal_match[] = {
	{ .compatible = "hisilicon,tsensor" },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, of_hisi_thermal_match);

static void hisi_thermal_toggle_sensor(struct hisi_thermal_sensor *sensor,
				       bool on)
{
	struct thermal_zone_device *tzd = sensor->tzd;

	tzd->ops->set_mode(tzd,
		on ? THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED);
}

static int hisi_thermal_probe(struct platform_device *pdev)
{
	struct hisi_thermal_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->thermal_lock);
	data->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->regs)) {
		dev_err(&pdev->dev, "failed to get io address\n");
		return PTR_ERR(data->regs);
	}

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0)
		return data->irq;

	platform_set_drvdata(pdev, data);

	data->clk = devm_clk_get(&pdev->dev, "thermal_clk");
	if (IS_ERR(data->clk)) {
		ret = PTR_ERR(data->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"failed to get thermal clk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable thermal clk: %d\n", ret);
		return ret;
	}


	return 0;
}

static int hisi_thermal_remove(struct platform_device *pdev)
{
	struct hisi_thermal_data *data = platform_get_drvdata(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hisi_thermal_suspend(struct device *dev)
{
	struct hisi_thermal_data *data = dev_get_drvdata(dev);


	return 0;
}

static int hisi_thermal_resume(struct device *dev)
{
	struct hisi_thermal_data *data = dev_get_drvdata(dev);
	int ret;
}
#endif

static SIMPLE_DEV_PM_OPS(hisi_thermal_pm_ops,
			 hisi_thermal_suspend, hisi_thermal_resume);

static struct platform_driver hisi_thermal_driver = {
	.driver = {
		.name		= "hisi_thermal",
		.pm		= &hisi_thermal_pm_ops,
		.of_match_table = of_hisi_thermal_match,
	},
	.probe	= hisi_thermal_probe,
	.remove	= hisi_thermal_remove,
};

module_platform_driver(hisi_thermal_driver);

MODULE_AUTHOR("Xinwei Kong <kong.kongxinwei@hisilicon.com>");
MODULE_AUTHOR("Leo Yan <leo.yan@linaro.org>");
MODULE_DESCRIPTION("Hisilicon thermal driver");
MODULE_LICENSE("GPL v2");
