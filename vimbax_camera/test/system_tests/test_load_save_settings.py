# Copyright 2024 Allied Vision Technologies GmbH. All Rights Reserved.
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

import os

import pytest
import warnings

import tempfile

from pathlib import Path
import xml.etree.ElementTree as ET

from vimbax_camera_msgs.srv import FeatureInfoQuery

from vimbax_camera_msgs.srv import FeatureAccessModeGet

from vimbax_camera_msgs.srv import SettingsLoadSave

from vimbax_camera_msgs.srv import FeatureFloatGet
from vimbax_camera_msgs.srv import FeatureFloatSet
from vimbax_camera_msgs.srv import FeatureFloatInfoGet

from conftest import vimbax_camera_node, camera_test_node_name, TestNode

from test_helper import check_error, filter_features, FeatureDataType


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_settings_save(test_node: TestNode, launch_context):
    settings_save_service = test_node.create_client(
        SettingsLoadSave, f"{camera_test_node_name}/settings/save")
    assert settings_save_service.wait_for_service(10)

    test_file_name = tempfile.mktemp(suffix=".xml")

    save_response = settings_save_service.call(SettingsLoadSave.Request(filename=test_file_name))
    check_error(save_response.error)
    assert Path(test_file_name).exists()
    # Test if xml is valid by parsing it
    ET.parse(test_file_name)

    os.remove(test_file_name)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_settings_save_load(test_node: TestNode, launch_context):
    settings_save_service = test_node.create_client(
        SettingsLoadSave, f"{camera_test_node_name}/settings/save")
    assert settings_save_service.wait_for_service(10)
    settings_load_service = test_node.create_client(
        SettingsLoadSave, f"{camera_test_node_name}/settings/load")
    assert settings_load_service.wait_for_service(10)

    test_file_name = tempfile.mktemp(suffix=".xml")

    save_response = settings_save_service.call(SettingsLoadSave.Request(filename=test_file_name))
    check_error(save_response.error)

    load_response = settings_load_service.call(SettingsLoadSave.Request(filename=test_file_name))
    check_error(load_response.error)

    os.remove(test_file_name)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_settings_save_load_float_value_change(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    settings_save_service = test_node.create_client(
        SettingsLoadSave, f"{camera_test_node_name}/settings/save")
    assert settings_save_service.wait_for_service(10)
    settings_load_service = test_node.create_client(
        SettingsLoadSave, f"{camera_test_node_name}/settings/load")
    assert settings_load_service.wait_for_service(10)
    feature_float_get_service = test_node.create_client(
        FeatureFloatGet, f"/{camera_test_node_name}/features/float_get")
    assert feature_float_get_service.wait_for_service(10)
    feature_float_set_service = test_node.create_client(
        FeatureFloatSet, f"/{camera_test_node_name}/features/float_set")
    assert feature_float_set_service.wait_for_service(10)
    feature_float_info_service = test_node.create_client(
        FeatureFloatInfoGet, f"/{camera_test_node_name}/features/float_info_get")
    assert feature_float_info_service.wait_for_service(10)

    test_file_name = tempfile.mktemp(suffix=".xml")

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = filter_features(
        feature_info_response.feature_info, feature_access_mode_service, FeatureDataType.FLOAT)

    default_values = {}

    for feature in available_features:
        default_value_response = feature_float_get_service.call(
            FeatureFloatGet.Request(feature_name=feature.name))
        check_error(default_value_response.error)

        default_values[feature.name] = default_value_response.value

    save_response = settings_save_service.call(SettingsLoadSave.Request(filename=test_file_name))
    check_error(save_response.error)

    for feature_name in default_values.keys():
        info_response = feature_float_info_service.call(
            FeatureFloatInfoGet.Request(feature_name=feature_name))
        if info_response.error.code == -30:
            warnings.warn(UserWarning(f"Skipping unavailable feature {feature_name}"))
            continue

        check_error(info_response.error)

        set_request = FeatureFloatSet.Request(feature_name=feature_name)
        set_request.value = info_response.min
        set_response = feature_float_set_service.call(set_request)
        check_error(set_response.error)

        set_get_request = FeatureFloatGet.Request(feature_name=feature_name)
        set_get_response = feature_float_get_service.call(set_get_request)
        check_error(set_get_response.error)
        assert set_get_response.value == info_response.min

    load_response = settings_load_service.call(SettingsLoadSave.Request(filename=test_file_name))
    check_error(load_response.error)

    for pair in default_values.items():
        load_get_request = FeatureFloatGet.Request(feature_name=pair[0])
        load_get_response = feature_float_get_service.call(load_get_request)
        check_error(load_get_response.error)
        assert load_get_response.value == pair[1]

    os.remove(test_file_name)