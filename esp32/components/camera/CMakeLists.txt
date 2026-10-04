# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(srcs "camera.c" "sscma_proto.c" "camera_sscma.c")
set(camera_ldfragments "")
if(CONFIG_MUSE_STACKCHAN_CAMERA)
    list(APPEND srcs "camera_gc0308.c")
    # jpge's static tables go to PSRAM on this build (see linker.lf).
    set(camera_ldfragments "linker.lf")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES esp_common esp_driver_spi
    PRIV_REQUIRES esp_pm esp_timer mbedtls
    LDFRAGMENTS ${camera_ldfragments}
)
