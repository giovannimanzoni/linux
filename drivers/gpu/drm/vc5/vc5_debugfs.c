/*
 *  Copyright © 2014 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/circ_buf.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <drm/drmP.h>

#include "vc5_drv.h"
#include "vc5_regs.h"

#define REGDEF(reg) { reg, #reg }
static const struct {
	uint32_t reg;
	const char *name;
} vc5_reg_defs[] = {
	REGDEF(V3D_HUB_AXICFG),
	REGDEF(V3D_HUB_UIFCFG),
	REGDEF(V3D_HUB_IDENT0),
	REGDEF(V3D_HUB_IDENT1),
	REGDEF(V3D_HUB_IDENT2),
	REGDEF(V3D_HUB_IDENT3),
	REGDEF(V3D_HUB_INT_STS),
	REGDEF(V3D_HUB_INT_MSK_STS),
	REGDEF(V3D_CTL_0_IDENT0),
	REGDEF(V3D_CTL_0_IDENT1),
	REGDEF(V3D_CTL_0_IDENT2),
	REGDEF(V3D_CTL_0_INT_STS),
	REGDEF(V3D_CTL_0_INT_MSK_STS),
	REGDEF(V3D_CLE_0_CT0CS),
	REGDEF(V3D_CLE_0_CT0CA),
	REGDEF(V3D_CLE_0_CT0EA),
	REGDEF(V3D_CLE_0_CT1CS),
	REGDEF(V3D_CLE_0_CT1CA),
	REGDEF(V3D_CLE_0_CT1EA),

	REGDEF(V3D_PTB_0_BPCA),
	REGDEF(V3D_PTB_0_BPCS),

	REGDEF(V3D_MMU_0_CTL),
	REGDEF(V3D_MMU_0_VIO_ADDR),

	REGDEF(V3D_GMP_0_STATUS),
	REGDEF(V3D_GMP_0_CFG),
	REGDEF(V3D_GMP_0_VIO_ADDR),

	REGDEF(V3D_GCA_SAFE_SHUTDOWN),
	REGDEF(V3D_GCA_SAFE_SHUTDOWN_ACK),
};

static int vc5_v3d_debugfs_regs(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(vc5_reg_defs); i++) {
		seq_printf(m, "%s (0x%04x): 0x%08x\n",
			   vc5_reg_defs[i].name, vc5_reg_defs[i].reg,
			   V3D_READ(vc5_reg_defs[i].reg));
	}

	return 0;
}

static int vc5_v3d_debugfs_ident(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	uint32_t ident0, ident1, ident2, ident3, cores;
	int ret, core;

	ret = pm_runtime_get_sync(vc5->dev);
	if (ret < 0)
		return ret;

	ident0 = V3D_READ(V3D_HUB_IDENT0);
	ident1 = V3D_READ(V3D_HUB_IDENT1);
	ident2 = V3D_READ(V3D_HUB_IDENT2);
	ident3 = V3D_READ(V3D_HUB_IDENT3);
	cores = VC5_GET_FIELD(ident1, V3D_HUB_IDENT1_NCORES);

	seq_printf(m, "Revision:   %d.%d.%d.%d\n",
		   VC5_GET_FIELD(ident1, V3D_HUB_IDENT1_TVER),
		   VC5_GET_FIELD(ident1, V3D_HUB_IDENT1_REV),
		   VC5_GET_FIELD(ident3, V3D_HUB_IDENT3_IPREV),
		   VC5_GET_FIELD(ident3, V3D_HUB_IDENT3_IPIDX));
	seq_printf(m, "MMU:        %s\n",
		   (ident2 & V3D_HUB_IDENT2_WITH_MMU) ? "yes" : "no");
	seq_printf(m, "TFU:        %s\n",
		   (ident1 & V3D_HUB_IDENT1_WITH_TFU) ? "yes" : "no");
	seq_printf(m, "TSY:        %s\n",
		   (ident1 & V3D_HUB_IDENT1_WITH_TSY) ? "yes" : "no");
	seq_printf(m, "MSO:        %s\n",
		   (ident1 & V3D_HUB_IDENT1_WITH_MSO) ? "yes" : "no");
	seq_printf(m, "L3C:        %s (%dkb)\n",
		   (ident1 & V3D_HUB_IDENT1_WITH_L3C) ? "yes" : "no",
		   VC5_GET_FIELD(ident2, V3D_HUB_IDENT2_L3C_NKB));

	for (core = 0; core < cores; core++) {
		uint32_t nslc, ntmu, qups;

		ident0 = V3D_READ(V3D_CTL_0_IDENT0 + core * V3D_CTL_CORE_OFFSET);
		ident1 = V3D_READ(V3D_CTL_0_IDENT1 + core * V3D_CTL_CORE_OFFSET);
		nslc = VC5_GET_FIELD(ident1, V3D_IDENT1_NSLC);
		ntmu = VC5_GET_FIELD(ident1, V3D_IDENT1_NTMU);
		qups = VC5_GET_FIELD(ident1, V3D_IDENT1_QUPS);

		seq_printf(m, "Core %d:\n", core);
		seq_printf(m, "  Revision:   %d.%d\n",
			   VC5_GET_FIELD(ident0, V3D_IDENT0_VER),
			   VC5_GET_FIELD(ident1, V3D_IDENT1_REV));
		seq_printf(m, "  Slices:     %d\n", nslc);
		seq_printf(m, "  TMUs:       %d\n", nslc * ntmu);
		seq_printf(m, "  QPUs:       %d\n", nslc * qups);
		seq_printf(m, "  Semaphores: %d\n",
			   VC5_GET_FIELD(ident1, V3D_IDENT1_NSEM));
	}

	pm_runtime_mark_last_busy(vc5->dev);
	pm_runtime_put_autosuspend(vc5->dev);

	return 0;
}

static int vc5_v3d_debugfs_jobs(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	unsigned long irqflags;
	struct vc5_exec_info *exec;

	spin_lock_irqsave(&vc5->job_lock, irqflags);

	seq_printf(m, "BCL jobs:\n");
	list_for_each_entry(exec, &vc5->bin_job_list, head) {
		seq_printf(m, "  BCL 0x%08x..0x%08x, RCL 0x%08x..0x%08x\n",
			   exec->ct0qba, exec->ct0qea,
			   exec->ct1qba, exec->ct1qea);
	}

	seq_printf(m, "RCL jobs:\n");
	list_for_each_entry(exec, &vc5->render_job_list, head) {
		seq_printf(m, "  BCL 0x%08x..0x%08x, RCL 0x%08x..0x%08x\n",
			   exec->ct0qba, exec->ct0qea,
			   exec->ct1qba, exec->ct1qea);
	}

	spin_unlock_irqrestore(&vc5->job_lock, irqflags);

	return 0;
}

static const struct drm_info_list vc5_debugfs_list[] = {
	{"v3d_ident", vc5_v3d_debugfs_ident, 0},
	{"v3d_regs", vc5_v3d_debugfs_regs, 0},
	{"v3d_jobs", vc5_v3d_debugfs_jobs, 0},
};

int
vc5_debugfs_init(struct drm_minor *minor)
{
	return drm_debugfs_create_files(vc5_debugfs_list,
					ARRAY_SIZE(vc5_debugfs_list),
					minor->debugfs_root, minor);
}
