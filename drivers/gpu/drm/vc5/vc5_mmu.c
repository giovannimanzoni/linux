/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * DOC: Broadcom VC5 MMU
 *
 * The VC5 hardware (compared to VC4) now includes an MMU.  It has a
 * single level of page tables for the V3D's 4GB address space to map
 * to AXI bus addresses, thus it could need up to 4MB of physically
 * contiguous memory to store the PTEs.
 *
 * Because the 4MB of contiguous memory for page tables is precious,
 * and switching between them is expensive, we load all BOs into the
 * same 4GB address space, and use the GMP to shield clients from each
 * other.  The GMP is an 8KB bitmask for memory protection at 128kb
 * granularity, which can be quickly re-loaded at context switch time.
 *
 * (XXX: Still need to do the GMP bits)
 */

#include "vc5_drv.h"
#include "vc5_regs.h"

static int vc5_mmu_flush_all(struct vc5_dev *vc5)
{
	int ret;

	/* Make sure that another flush isn't already running when we
	 * start this one.
	 */
	ret = wait_for(!(V3D_READ(V3D_MMU_0_CTL) &
			 V3D_MMU_CTL_TLB_CLEARING), 100);
	if (ret)
		dev_err(vc5->dev, "TLB clear wait idle failed\n");

	V3D_WRITE(V3D_MMU_0_CTL,
		  V3D_MMU_CTL_ENABLE |
		  V3D_MMU_CTL_TLB_CLEAR);

	V3D_WRITE(V3D_MMUC_CONTROL,
		  V3D_MMUC_CONTROL_FLUSH |
		  V3D_MMUC_CONTROL_ENABLE);

	ret = wait_for(!(V3D_READ(V3D_MMU_0_CTL) &
			 V3D_MMU_CTL_TLB_CLEARING), 100);
	if (ret)
		return ret;

	ret = wait_for(!(V3D_READ(V3D_MMUC_CONTROL) &
			 V3D_MMUC_CONTROL_FLUSHING), 100);

	return ret;
}

int vc5_mmu_set_page_table(struct vc5_dev *vc5)
{
	V3D_WRITE(V3D_MMU_0_PT_PA_BASE, vc5->pt_paddr >> PAGE_SHIFT);
	V3D_WRITE(V3D_MMU_0_CTL, V3D_MMU_CTL_ENABLE);
	V3D_WRITE(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_ENABLE);

	return vc5_mmu_flush_all(vc5);
}

void vc5_mmu_insert_ptes(struct vc5_bo *bo)
{
	struct vc5_dev *vc5 = to_vc5_dev(bo->base.dev);
	u32 page = bo->node.start;
	u32 page_prot = BIT(28) | BIT(29);
	unsigned count;
	struct scatterlist *sgl;

	for_each_sg(bo->sgt->sgl, sgl, bo->sgt->nents, count) {
		u32 pte = page_prot | (sg_dma_address(sgl) >> PAGE_SHIFT);
		u32 i;

		for (i = 0; i < sg_dma_len(sgl) >> PAGE_SHIFT; i++)
			vc5->pt[page++] = pte + i;
	}

	if (vc5_mmu_flush_all(vc5))
		dev_err(vc5->dev, "MMU flush timeout\n");
}

void vc5_mmu_remove_ptes(struct vc5_bo *bo)
{
	struct vc5_dev *vc5 = to_vc5_dev(bo->base.dev);
	u32 npages = bo->base.size >> PAGE_SHIFT;
	u32 page;

	for (page = bo->node.start; page < bo->node.start + npages; page++)
		vc5->pt[page] = 0;

	if (vc5_mmu_flush_all(vc5))
		dev_err(vc5->dev, "MMU flush timeout\n");
}
