/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @brief File containing API definitions for the
 * FMAC IF Layer of the Wi-Fi driver.
 */

#include "host_rpu_umac_if.h"
#include "hal_api.h"
#include "fmac_api.h"
#include "fmac_cmd.h"
#include <stdio.h>
static const int valid_rate[] = { 1, 2, 55, 11, 6, 9, 12, 18, 24, 36, 48, 54, 0, 1, 2, 3, 4, 5, 6, 7, -1 };

struct nrf_wifi_fmac_priv *nrf_wifi_fmac_off_raw_tx_init(void)
{
	struct nrf_wifi_fmac_priv *fpriv = NULL;
	struct nrf_wifi_hal_cfg_params hal_cfg_params;

	fpriv = nrf_wifi_osal_mem_zalloc(sizeof(*fpriv));
	if (!fpriv) {
		nrf_wifi_osal_log_err("%s: Unable to allocate fpriv",
				      __func__);
		goto out;
	}

	nrf_wifi_osal_mem_set(&hal_cfg_params,
			      0,
			      sizeof(hal_cfg_params));

	hal_cfg_params.max_cmd_size = MAX_NRF_WIFI_UMAC_CMD_SIZE;
	hal_cfg_params.max_event_size = MAX_EVENT_POOL_LEN;

	fpriv->hpriv = nrf_wifi_hal_init(&hal_cfg_params,
					 &nrf_wifi_fmac_event_callback,
					 NULL);
	if (!fpriv->hpriv) {
		nrf_wifi_osal_log_err("%s: Unable to do HAL init",
				      __func__);
		nrf_wifi_osal_mem_free(fpriv);
		fpriv = NULL;
		goto out;
	}
out:
	return fpriv;
}

static enum nrf_wifi_status nrf_wifi_fmac_off_raw_tx_fw_init(
		struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx,
		struct nrf_wifi_phy_rf_params *rf_params,
		bool rf_params_valid,
#ifdef NRF_WIFI_LOW_POWER
		int sleep_type,
#endif /* NRF_WIFI_LOW_POWER */
		unsigned int phy_calib,
		enum op_band op_band,
		bool beamforming,
		struct nrf_wifi_tx_pwr_ctrl_params *tx_pwr_ctrl,
		struct nrf_wifi_board_params *board_params)
{
	unsigned long start_time_us = 0;
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;

	if (!fmac_dev_ctx) {
		nrf_wifi_osal_log_err("%s: Invalid device context",
				      __func__);
		goto out;
	}

	status = umac_cmd_init(fmac_dev_ctx,
			       rf_params,
			       rf_params_valid,
#ifdef NRF_WIFI_LOW_POWER
			       sleep_type,
#endif /* NRF_WIFI_LOW_POWER */
			       phy_calib,
			       op_band,
			       beamforming,
			       tx_pwr_ctrl,
			       board_params);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: UMAC init failed",
				      __func__);
		goto out;
	}

	start_time_us = nrf_wifi_osal_time_get_curr_us();
	while (!fmac_dev_ctx->fw_init_done) {
		nrf_wifi_osal_sleep_ms(1);
#define MAX_INIT_WAIT (5 * 1000 * 1000)
		if (nrf_wifi_osal_time_elapsed_us(start_time_us) >= MAX_INIT_WAIT) {
			break;
		}
	}

	if (!fmac_dev_ctx->fw_init_done) {
		nrf_wifi_osal_log_err("%s: UMAC init timed out",
				      __func__);
		status = NRF_WIFI_STATUS_FAIL;
		goto out;
	}

	status = NRF_WIFI_STATUS_SUCCESS;
out:
	return status;
}

enum nrf_wifi_status nrf_wifi_fmac_off_raw_tx_dev_init(
		struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx,
#ifdef NRF_WIFI_LOW_POWER
		int sleep_type,
#endif /* NRF_WIFI_LOW_POWER */
		unsigned int phy_calib,
		enum op_band op_band,
		bool beamforming,
		struct nrf_wifi_tx_pwr_ctrl_params *tx_pwr_ctrl_params,
		struct nrf_wifi_tx_pwr_ceil_params *tx_pwr_ceil_params,
		struct nrf_wifi_board_params *board_params)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_fmac_otp_info otp_info;
	struct nrf_wifi_phy_rf_params phy_rf_params;

	if (!fmac_dev_ctx) {
		nrf_wifi_osal_log_err("%s: Invalid device context",
				      __func__);
		goto out;
	}

	status = nrf_wifi_hal_dev_init(fmac_dev_ctx->hal_dev_ctx);
	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: nrf_wifi_hal_dev_init failed",
				      __func__);
		goto out;
	}

	fmac_dev_ctx->tx_pwr_ceil_params = nrf_wifi_osal_mem_alloc(sizeof(*tx_pwr_ceil_params));
	nrf_wifi_osal_mem_cpy(fmac_dev_ctx->tx_pwr_ceil_params,
			      tx_pwr_ceil_params,
			      sizeof(*tx_pwr_ceil_params));

	nrf_wifi_osal_mem_set(&otp_info,
			      0xFF,
			      sizeof(otp_info));

	status = nrf_wifi_hal_otp_info_get(fmac_dev_ctx->hal_dev_ctx,
					   &otp_info.info,
					   &otp_info.flags);
	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: Fetching of RPU OTP information failed",
				      __func__);
		goto out;
	}

	status = nrf_wifi_fmac_rf_params_get(fmac_dev_ctx,
		                             &phy_rf_params);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: RF parameters get failed",
				     __func__);
		goto out;
	}

	status = nrf_wifi_fmac_off_raw_tx_fw_init(fmac_dev_ctx,
						  &phy_rf_params,
						  true,
#ifdef NRF_WIFI_LOW_POWER
						  sleep_type,
#endif /* NRF_WIFI_LOW_POWER */
						  phy_calib,
						  op_band,
						  beamforming,
						  tx_pwr_ctrl_params,
						  board_params);

	if (status == NRF_WIFI_STATUS_FAIL) {
		nrf_wifi_osal_log_err("%s: nrf_wifi_fmac_off_raw_tx_fw_init failed",
				      __func__);
		goto out;
	}
out:
	return status;
}

void nrf_wifi_fmac_off_raw_tx_dev_rem(struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx)
{
	nrf_wifi_hal_dev_rem(fmac_dev_ctx->hal_dev_ctx);
	nrf_wifi_osal_mem_free(fmac_dev_ctx);
}

void nrf_wifi_fmac_off_raw_tx_dev_deinit(struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx)
{
	nrf_wifi_osal_mem_free(fmac_dev_ctx->tx_pwr_ceil_params);
}

void nrf_wifi_fmac_off_raw_tx_deinit(struct nrf_wifi_fmac_priv *fpriv)
{
	nrf_wifi_hal_deinit(fpriv->hpriv);
	nrf_wifi_osal_mem_free(fpriv);
}

static bool valid_rates(enum nrf_wifi_off_raw_tx_tput_mode tput_mode,
			enum nrf_wifi_off_raw_tx_rate rate)
{
	if (tput_mode == TPUT_MODE_LEGACY){
		if (rate > RATE_54M) {
			return false;
		}
	} else {
		if (rate <= RATE_54M) {
			return false;
		}
	}

	return true;
}

enum nrf_wifi_status nrf_wifi_fmac_off_raw_tx_conf(struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx,
					           struct nrf_wifi_off_raw_tx_conf *conf)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_offload_ctrl_params *off_ctrl_params = NULL;
	struct nrf_wifi_offload_tx_ctrl *off_tx_params = NULL;

	if (!conf || !fmac_dev_ctx) {
		nrf_wifi_osal_log_err("%s: Invalid params", __func__);
		goto out;
	}

	off_ctrl_params = nrf_wifi_osal_mem_zalloc(sizeof(*off_ctrl_params));
	if (!off_ctrl_params) {
		nrf_wifi_osal_log_err("%s: Failed to allocate memory for off_ctrl_params", __func__);
		goto out;
	}

	off_tx_params = nrf_wifi_osal_mem_zalloc(sizeof(*off_tx_params));
	if (!off_tx_params) {
		nrf_wifi_osal_log_err("%s Failed to allocate memory for off_tx_params: ", __func__);
		goto out;
	}

	if(!valid_rates(conf->tput_mode, conf->rate)) {
		nrf_wifi_osal_log_err("%s Invalid rate. Throughput mode: %d, rate: %d\n", __func__,
				      conf->tput_mode, conf->rate);
		goto out;
	}

	off_ctrl_params->channel_no = conf->chan;
	off_ctrl_params->period_in_us = conf->period_us;
	off_ctrl_params->tx_pwr = conf->tx_pwr;
	off_tx_params->he_gi_type = conf->he_gi;
	off_tx_params->he_ltf = conf->he_ltf;
	off_tx_params->pkt_ram_ptr = RPU_MEM_PKT_BASE;
	off_tx_params->pkt_length = conf->pkt_len;
	off_tx_params->rate_flags = conf->tput_mode;
	off_tx_params->rate = valid_rate[conf->rate];
	off_tx_params->rate_preamble_type = conf->short_preamble;
	off_tx_params->rate_retries = conf->num_retries;

	status = hal_rpu_mem_write(fmac_dev_ctx->hal_dev_ctx,
				   RPU_MEM_PKT_BASE,
				   conf->pkt,
				   conf->pkt_len);
	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: hal_rpu_mem_write failed", __func__);
		goto out;
	}

	status = umac_cmd_off_raw_tx_conf(fmac_dev_ctx,
					  off_ctrl_params,
					  off_tx_params);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: umac_cmd_offload_raw_tx_conf failed", __func__);
		goto out;
	}
out:
	return status;
}

enum nrf_wifi_status nrf_wifi_fmac_off_raw_tx_start(struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;

	status = umac_cmd_off_raw_tx_ctrl(fmac_dev_ctx, 1);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: umac_cmd_off_raw_tx_ctrl failed", __func__);
		goto out;
	}
out:
	return status;
}

enum nrf_wifi_status nrf_wifi_fmac_off_raw_tx_stop(struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;

	status = umac_cmd_off_raw_tx_ctrl(fmac_dev_ctx, 0);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		nrf_wifi_osal_log_err("%s: umac_cmd_offload_raw_tx_ctrl failed", __func__);
		goto out;
	}
out:
	return status;
}
