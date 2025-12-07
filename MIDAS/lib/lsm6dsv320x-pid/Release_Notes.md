---
pagetitle: Release Notes for LSM6DSV320X Component
lang: en
header-includes: <link rel="icon" type="image/x-icon" href="_htmresc/favicon.png" />
---

::: {.row}
::: {.col-sm-12 .col-lg-4}

<center>
# Release Notes for LSM6DSV320X Component Driver
Copyright &copy; 2025 STMicroelectronics\

[![ST logo](_htmresc/st_logo_2020.png)](https://www.st.com){.logo}
</center>

# License

This software component is licensed by ST under BSD 3-Clause license, the "License".
You may not use this component except in compliance with the License. You may obtain a copy of the License at:

[BSD 3-Clause license](https://opensource.org/licenses/BSD-3-Clause)

# Purpose

This directory contains the LSM6DSV320X component drivers.
:::

::: {.col-sm-12 .col-lg-8}
# Update history

::: {.collapse}
<input type="checkbox" id="collapse-section1" aria-hidden="true">
<label for="collapse-section1" aria-hidden="true">V1.0.0 / 04-Apr-2025</label>
<div>

## Main changes

### First release

- First official release [ref. DS v1.0]

##

</div>

<input type="checkbox" id="collapse-section2" aria-hidden="true">
<label for="collapse-section2" aria-hidden="true">V1.1.0 / 07-Jul-2025</label>
<div>

## Main changes

- Fix driver formatting options
- Added pointer to private data in stmdev_ctx_t
- Fix setting of few embedded registers
- Fix wrong ODR enums values for HA02 and add HA03
- Fix wrong address for register EMB_FUNC_SENSOR_CONV_EN
- Set ois_drdy parameter in filt_settling_mask_get()
- add from_f16_to_f32 API
- fix fifo_fsm_batch_set/get APIs
- Fix typo in fifo_out_raw_get() API

##

</div>

<input type="checkbox" id="collapse-section3" aria-hidden="true">
<label for="collapse-section3" aria-hidden="true">V2.0.0 / 17-Sep-2025</label>
<div>

## Main changes

- Added from_fs125_to_mdps() conversion API
- Added checks before writes to avoid random content from being written
- Fixed reset_set API wrong boot and sw_reset settings
- Added fifo_event_t enum for stop_on_wtm_set/get functions
- Fix fifo_batch_counter_threshold_set/get
- Adding CODE_OF_CONDUCT.md and SECURITY.md

##

</div>

<input type="checkbox" id="collapse-section4" aria-hidden="true">
<label for="collapse-section4" aria-hidden="true">V3.0.0 / 07-Oct-2025</label>
<div>

## Main changes

- Add routine to reset SFLP game rotation logic

##

</div>

<input type="checkbox" id="collapse-section5" checked aria-hidden="true">
<label for="collapse-section5" aria-hidden="true">V4.0.0 / 05-Nov-2025</label>
<div>

## Main changes

- Upgrade reset APIs

##

</div>
:::


:::
:::

<footer class="sticky">
::: {.columns}
::: {.column width="95%"}
For complete documentation on LSM6DSV320X,
visit:
[LSM6DSV320X](https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsv320x.html)
:::
::: {.column width="5%"}
<abbr title="Based on template cx566953 version 1.0">Info</abbr>
:::
:::
</footer>
