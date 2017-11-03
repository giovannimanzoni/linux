/*
 * Copyright (C) 2014-2015 Broadcom
 * Copyright (C) 2013 Red Hat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * DOC: Broadcom VC5 Graphics Driver
 *
 * The Broadcom V3D V3D 3.3 (present in BCM7268, aka vc5 3D) contains
 * a OpenGL ES 3.1-compatible 3D engine.  This driver supports that 3D
 * engine, while other VC5 components (such as an updated display
 * pipeline) will continue to be supported by the VC4 driver.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>

#include "uapi/drm/vc5_drm.h"
#include "vc5_drv.h"
#include "vc5_regs.h"

#define DRIVER_NAME "vc5"
#define DRIVER_DESC "Broadcom VC5 graphics"
#define DRIVER_DATE "20170823"
#define DRIVER_MAJOR 0
#define DRIVER_MINOR 0
#define DRIVER_PATCHLEVEL 0

#ifdef CONFIG_PM
static int vc5_runtime_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct vc5_dev *vc5 = to_vc5_dev(drm);

	vc5_irq_disable(vc5);

	/* XXX
	clk_disable_unprepare(v3d->clk);
	*/

	return 0;
}

static int vc5_runtime_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct vc5_dev *vc5 = to_vc5_dev(drm);

	/* XXX
	ret = clk_prepare_enable(v3d->clk);
	if (ret != 0)
		return ret;
	*/

	/* XXX: VPM base */

	vc5_mmu_set_page_table(vc5);
	vc5_irq_enable(vc5);

	return 0;
}
#endif

static const struct dev_pm_ops vc5_v3d_pm_ops = {
	SET_RUNTIME_PM_OPS(vc5_runtime_suspend, vc5_runtime_resume, NULL)
};

static int vc5_get_param_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	struct drm_vc5_get_param *args = data;
	int ret;
	static const uint32_t reg_map[] = {
		[DRM_VC5_PARAM_V3D_UIFCFG] = V3D_HUB_UIFCFG,
		[DRM_VC5_PARAM_V3D_HUB_IDENT1] = V3D_HUB_IDENT1,
		[DRM_VC5_PARAM_V3D_HUB_IDENT2] = V3D_HUB_IDENT2,
		[DRM_VC5_PARAM_V3D_HUB_IDENT3] = V3D_HUB_IDENT3,
		[DRM_VC5_PARAM_V3D_CORE0_IDENT0] = V3D_CTL_0_IDENT0,
		[DRM_VC5_PARAM_V3D_CORE0_IDENT1] = V3D_CTL_0_IDENT1,
		[DRM_VC5_PARAM_V3D_CORE0_IDENT2] = V3D_CTL_0_IDENT2,
	};

	if (args->pad != 0)
		return -EINVAL;

	/* Register 0 is AXICFG, which we don't expose, so a sparse
	 * map would be fine.
	 */
	if (args->param < ARRAY_SIZE(reg_map) && reg_map[args->param]) {
		ret = pm_runtime_get_sync(vc5->dev);
		args->value = V3D_READ(reg_map[args->param]);
		pm_runtime_mark_last_busy(vc5->dev);
		pm_runtime_put_autosuspend(vc5->dev);
		return 0;
	}

	/* Any params that aren't just register reads would go here. */

	DRM_DEBUG("Unknown parameter %d\n", args->param);
	return -EINVAL;
}

static const struct file_operations vc5_drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = vc5_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.compat_ioctl = drm_compat_ioctl,
	.llseek = noop_llseek,
};

static const struct drm_ioctl_desc vc5_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(VC5_SUBMIT_CL, vc5_submit_cl_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(VC5_WAIT_SEQNO, vc5_wait_seqno_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(VC5_WAIT_BO, vc5_wait_bo_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(VC5_CREATE_BO, vc5_create_bo_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(VC5_MMAP_BO, vc5_mmap_bo_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(VC5_GET_PARAM, vc5_get_param_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(VC5_GET_BO_OFFSET, vc5_get_bo_offset_ioctl, DRM_RENDER_ALLOW),
};

static const struct vm_operations_struct vc5_vm_ops = {
	.fault = vc5_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct drm_driver vc5_drm_driver = {
	.driver_features = (DRIVER_GEM |
			    DRIVER_RENDER |
			    DRIVER_PRIME),

#if defined(CONFIG_DEBUG_FS)
	.debugfs_init = vc5_debugfs_init,
#endif

	.gem_free_object_unlocked = vc5_free_object,
	.gem_vm_ops = &vc5_vm_ops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_res_obj = vc5_prime_res_obj,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = vc5_prime_import_sg_table,
	.gem_prime_vmap = vc5_prime_vmap,
	.gem_prime_vunmap = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap = vc5_prime_mmap,

	.ioctls = vc5_drm_ioctls,
	.num_ioctls = ARRAY_SIZE(vc5_drm_ioctls),
	.fops = &vc5_drm_fops,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static int vc5_platform_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_device *drm;
	struct vc5_dev *vc5;
	int ret = 0;
	struct resource *res;

	dev->coherent_dma_mask = DMA_BIT_MASK(32);

	vc5 = devm_kzalloc(dev, sizeof(*vc5), GFP_KERNEL);
	if (!vc5)
		return -ENOMEM;
	vc5->dev = dev;
	vc5->pdev = pdev;

	res = platform_get_resource(vc5->pdev, IORESOURCE_MEM, 0);
	vc5->regs = devm_ioremap_resource(vc5->dev, res);
	if (IS_ERR(vc5->regs)) {
		DRM_ERROR("Failed to map registers: %ld\n", PTR_ERR(vc5->regs));
		return PTR_ERR(vc5->regs);
	}

	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 50);
	pm_runtime_enable(dev);

	drm = drm_dev_alloc(&vc5_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	platform_set_drvdata(pdev, drm);
	vc5->drm = drm;
	drm->dev_private = vc5;

	ret = vc5_gem_init(drm);
	if (ret)
		goto dev_destroy;

	vc5_irq_init(vc5);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto gem_destroy;

	return 0;

gem_destroy:
	vc5_gem_destroy(drm);
dev_destroy:
	drm_dev_put(drm);
	return ret;
}

static int vc5_platform_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);

	vc5_gem_destroy(drm);

	drm_dev_put(drm);

	return 0;
}

static const struct of_device_id vc5_of_match[] = {
	{ .compatible = "brcm,7268-v3d", },
	{},
};
MODULE_DEVICE_TABLE(of, vc5_of_match);

static struct platform_driver vc5_platform_driver = {
	.probe		= vc5_platform_drm_probe,
	.remove		= vc5_platform_drm_remove,
	.driver		= {
		.name	= "vc5",
		.of_match_table = vc5_of_match,
	},
};

static int __init vc5_drm_register(void)
{
	return platform_driver_register(&vc5_platform_driver);
}

static void __exit vc5_drm_unregister(void)
{
	platform_driver_unregister(&vc5_platform_driver);
}

module_init(vc5_drm_register);
module_exit(vc5_drm_unregister);

MODULE_ALIAS("platform:vc5-drm");
MODULE_DESCRIPTION("Broadcom VC5 DRM Driver");
MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_LICENSE("GPL v2");
