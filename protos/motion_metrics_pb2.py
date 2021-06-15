# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: motion_metrics.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import scenario_pb2 as scenario__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='motion_metrics.proto',
  package='waymo.open_dataset',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x14motion_metrics.proto\x12\x12waymo.open_dataset\x1a\x0escenario.proto\"Q\n\x10SingleTrajectory\x12\x11\n\tobject_id\x18\x01 \x01(\x05\x12\x14\n\x08\x63\x65nter_x\x18\x02 \x03(\x02\x42\x02\x10\x01\x12\x14\n\x08\x63\x65nter_y\x18\x03 \x03(\x02\x42\x02\x10\x01\"c\n\x11JointTrajectories\x12:\n\x0ctrajectories\x18\x02 \x03(\x0b\x32$.waymo.open_dataset.SingleTrajectory\x12\x12\n\nconfidence\x18\x03 \x01(\x02\"X\n\x14MultimodalPrediction\x12@\n\x11joint_predictions\x18\x01 \x03(\x0b\x32%.waymo.open_dataset.JointTrajectories\"u\n\x13ScenarioPredictions\x12\x13\n\x0bscenario_id\x18\x01 \x01(\t\x12I\n\x17multi_modal_predictions\x18\x02 \x03(\x0b\x32(.waymo.open_dataset.MultimodalPrediction\"\xd7\x01\n\x13MotionMetricsBundle\x12;\n\robject_filter\x18\x07 \x01(\x0e\x32$.waymo.open_dataset.Track.ObjectType\x12\x18\n\x10measurement_step\x18\x06 \x01(\x05\x12\x0f\n\x07min_ade\x18\x01 \x01(\x02\x12\x0f\n\x07min_fde\x18\x02 \x01(\x02\x12\x11\n\tmiss_rate\x18\x03 \x01(\x02\x12\x14\n\x0coverlap_rate\x18\x04 \x01(\x02\x12\x1e\n\x16mean_average_precision\x18\x05 \x01(\x02\"Q\n\rMotionMetrics\x12@\n\x0fmetrics_bundles\x18\x01 \x03(\x0b\x32\'.waymo.open_dataset.MotionMetricsBundle\"\x93\x04\n\x13MotionMetricsConfig\x12\"\n\x16track_steps_per_second\x18\x01 \x01(\x05:\x02\x31\x30\x12&\n\x1bprediction_steps_per_second\x18\x02 \x01(\x05:\x01\x32\x12!\n\x15track_history_samples\x18\x03 \x01(\x05:\x02\x31\x30\x12 \n\x14track_future_samples\x18\x04 \x01(\x05:\x02\x38\x30\x12\x1e\n\x11speed_lower_bound\x18\x05 \x01(\x02:\x03\x31.4\x12\x1d\n\x11speed_upper_bound\x18\x06 \x01(\x02:\x02\x31\x31\x12\x1e\n\x11speed_scale_lower\x18\x07 \x01(\x02:\x03\x30.5\x12\x1c\n\x11speed_scale_upper\x18\x08 \x01(\x02:\x01\x31\x12Z\n\x13step_configurations\x18\t \x03(\x0b\x32=.waymo.open_dataset.MotionMetricsConfig.MeasurementStepConfig\x12\x1a\n\x0fmax_predictions\x18\n \x01(\x05:\x01\x36\x1av\n\x15MeasurementStepConfig\x12\x18\n\x10measurement_step\x18\x01 \x01(\x05\x12\x1e\n\x16lateral_miss_threshold\x18\x02 \x01(\x02\x12#\n\x1blongitudinal_miss_threshold\x18\x03 \x01(\x02'
  ,
  dependencies=[scenario__pb2.DESCRIPTOR,])




_SINGLETRAJECTORY = _descriptor.Descriptor(
  name='SingleTrajectory',
  full_name='waymo.open_dataset.SingleTrajectory',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='object_id', full_name='waymo.open_dataset.SingleTrajectory.object_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center_x', full_name='waymo.open_dataset.SingleTrajectory.center_x', index=1,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\020\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center_y', full_name='waymo.open_dataset.SingleTrajectory.center_y', index=2,
      number=3, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\020\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=60,
  serialized_end=141,
)


_JOINTTRAJECTORIES = _descriptor.Descriptor(
  name='JointTrajectories',
  full_name='waymo.open_dataset.JointTrajectories',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='trajectories', full_name='waymo.open_dataset.JointTrajectories.trajectories', index=0,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='confidence', full_name='waymo.open_dataset.JointTrajectories.confidence', index=1,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=143,
  serialized_end=242,
)


_MULTIMODALPREDICTION = _descriptor.Descriptor(
  name='MultimodalPrediction',
  full_name='waymo.open_dataset.MultimodalPrediction',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='joint_predictions', full_name='waymo.open_dataset.MultimodalPrediction.joint_predictions', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=244,
  serialized_end=332,
)


_SCENARIOPREDICTIONS = _descriptor.Descriptor(
  name='ScenarioPredictions',
  full_name='waymo.open_dataset.ScenarioPredictions',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='scenario_id', full_name='waymo.open_dataset.ScenarioPredictions.scenario_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='multi_modal_predictions', full_name='waymo.open_dataset.ScenarioPredictions.multi_modal_predictions', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=334,
  serialized_end=451,
)


_MOTIONMETRICSBUNDLE = _descriptor.Descriptor(
  name='MotionMetricsBundle',
  full_name='waymo.open_dataset.MotionMetricsBundle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='object_filter', full_name='waymo.open_dataset.MotionMetricsBundle.object_filter', index=0,
      number=7, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='measurement_step', full_name='waymo.open_dataset.MotionMetricsBundle.measurement_step', index=1,
      number=6, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='min_ade', full_name='waymo.open_dataset.MotionMetricsBundle.min_ade', index=2,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='min_fde', full_name='waymo.open_dataset.MotionMetricsBundle.min_fde', index=3,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='miss_rate', full_name='waymo.open_dataset.MotionMetricsBundle.miss_rate', index=4,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='overlap_rate', full_name='waymo.open_dataset.MotionMetricsBundle.overlap_rate', index=5,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mean_average_precision', full_name='waymo.open_dataset.MotionMetricsBundle.mean_average_precision', index=6,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=454,
  serialized_end=669,
)


_MOTIONMETRICS = _descriptor.Descriptor(
  name='MotionMetrics',
  full_name='waymo.open_dataset.MotionMetrics',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='metrics_bundles', full_name='waymo.open_dataset.MotionMetrics.metrics_bundles', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=671,
  serialized_end=752,
)


_MOTIONMETRICSCONFIG_MEASUREMENTSTEPCONFIG = _descriptor.Descriptor(
  name='MeasurementStepConfig',
  full_name='waymo.open_dataset.MotionMetricsConfig.MeasurementStepConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='measurement_step', full_name='waymo.open_dataset.MotionMetricsConfig.MeasurementStepConfig.measurement_step', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='lateral_miss_threshold', full_name='waymo.open_dataset.MotionMetricsConfig.MeasurementStepConfig.lateral_miss_threshold', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='longitudinal_miss_threshold', full_name='waymo.open_dataset.MotionMetricsConfig.MeasurementStepConfig.longitudinal_miss_threshold', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1168,
  serialized_end=1286,
)

_MOTIONMETRICSCONFIG = _descriptor.Descriptor(
  name='MotionMetricsConfig',
  full_name='waymo.open_dataset.MotionMetricsConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='track_steps_per_second', full_name='waymo.open_dataset.MotionMetricsConfig.track_steps_per_second', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=10,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='prediction_steps_per_second', full_name='waymo.open_dataset.MotionMetricsConfig.prediction_steps_per_second', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=2,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='track_history_samples', full_name='waymo.open_dataset.MotionMetricsConfig.track_history_samples', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=10,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='track_future_samples', full_name='waymo.open_dataset.MotionMetricsConfig.track_future_samples', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=80,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_lower_bound', full_name='waymo.open_dataset.MotionMetricsConfig.speed_lower_bound', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(1.4),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_upper_bound', full_name='waymo.open_dataset.MotionMetricsConfig.speed_upper_bound', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(11),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_scale_lower', full_name='waymo.open_dataset.MotionMetricsConfig.speed_scale_lower', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(0.5),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_scale_upper', full_name='waymo.open_dataset.MotionMetricsConfig.speed_scale_upper', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='step_configurations', full_name='waymo.open_dataset.MotionMetricsConfig.step_configurations', index=8,
      number=9, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_predictions', full_name='waymo.open_dataset.MotionMetricsConfig.max_predictions', index=9,
      number=10, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=6,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_MOTIONMETRICSCONFIG_MEASUREMENTSTEPCONFIG, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=755,
  serialized_end=1286,
)

_JOINTTRAJECTORIES.fields_by_name['trajectories'].message_type = _SINGLETRAJECTORY
_MULTIMODALPREDICTION.fields_by_name['joint_predictions'].message_type = _JOINTTRAJECTORIES
_SCENARIOPREDICTIONS.fields_by_name['multi_modal_predictions'].message_type = _MULTIMODALPREDICTION
_MOTIONMETRICSBUNDLE.fields_by_name['object_filter'].enum_type = scenario__pb2._TRACK_OBJECTTYPE
_MOTIONMETRICS.fields_by_name['metrics_bundles'].message_type = _MOTIONMETRICSBUNDLE
_MOTIONMETRICSCONFIG_MEASUREMENTSTEPCONFIG.containing_type = _MOTIONMETRICSCONFIG
_MOTIONMETRICSCONFIG.fields_by_name['step_configurations'].message_type = _MOTIONMETRICSCONFIG_MEASUREMENTSTEPCONFIG
DESCRIPTOR.message_types_by_name['SingleTrajectory'] = _SINGLETRAJECTORY
DESCRIPTOR.message_types_by_name['JointTrajectories'] = _JOINTTRAJECTORIES
DESCRIPTOR.message_types_by_name['MultimodalPrediction'] = _MULTIMODALPREDICTION
DESCRIPTOR.message_types_by_name['ScenarioPredictions'] = _SCENARIOPREDICTIONS
DESCRIPTOR.message_types_by_name['MotionMetricsBundle'] = _MOTIONMETRICSBUNDLE
DESCRIPTOR.message_types_by_name['MotionMetrics'] = _MOTIONMETRICS
DESCRIPTOR.message_types_by_name['MotionMetricsConfig'] = _MOTIONMETRICSCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SingleTrajectory = _reflection.GeneratedProtocolMessageType('SingleTrajectory', (_message.Message,), {
  'DESCRIPTOR' : _SINGLETRAJECTORY,
  '__module__' : 'motion_metrics_pb2'
  # @@protoc_insertion_point(class_scope:waymo.open_dataset.SingleTrajectory)
  })
_sym_db.RegisterMessage(SingleTrajectory)

JointTrajectories = _reflection.GeneratedProtocolMessageType('JointTrajectories', (_message.Message,), {
  'DESCRIPTOR' : _JOINTTRAJECTORIES,
  '__module__' : 'motion_metrics_pb2'
  # @@protoc_insertion_point(class_scope:waymo.open_dataset.JointTrajectories)
  })
_sym_db.RegisterMessage(JointTrajectories)

MultimodalPrediction = _reflection.GeneratedProtocolMessageType('MultimodalPrediction', (_message.Message,), {
  'DESCRIPTOR' : _MULTIMODALPREDICTION,
  '__module__' : 'motion_metrics_pb2'
  # @@protoc_insertion_point(class_scope:waymo.open_dataset.MultimodalPrediction)
  })
_sym_db.RegisterMessage(MultimodalPrediction)

ScenarioPredictions = _reflection.GeneratedProtocolMessageType('ScenarioPredictions', (_message.Message,), {
  'DESCRIPTOR' : _SCENARIOPREDICTIONS,
  '__module__' : 'motion_metrics_pb2'
  # @@protoc_insertion_point(class_scope:waymo.open_dataset.ScenarioPredictions)
  })
_sym_db.RegisterMessage(ScenarioPredictions)

MotionMetricsBundle = _reflection.GeneratedProtocolMessageType('MotionMetricsBundle', (_message.Message,), {
  'DESCRIPTOR' : _MOTIONMETRICSBUNDLE,
  '__module__' : 'motion_metrics_pb2'
  # @@protoc_insertion_point(class_scope:waymo.open_dataset.MotionMetricsBundle)
  })
_sym_db.RegisterMessage(MotionMetricsBundle)

MotionMetrics = _reflection.GeneratedProtocolMessageType('MotionMetrics', (_message.Message,), {
  'DESCRIPTOR' : _MOTIONMETRICS,
  '__module__' : 'motion_metrics_pb2'
  # @@protoc_insertion_point(class_scope:waymo.open_dataset.MotionMetrics)
  })
_sym_db.RegisterMessage(MotionMetrics)

MotionMetricsConfig = _reflection.GeneratedProtocolMessageType('MotionMetricsConfig', (_message.Message,), {

  'MeasurementStepConfig' : _reflection.GeneratedProtocolMessageType('MeasurementStepConfig', (_message.Message,), {
    'DESCRIPTOR' : _MOTIONMETRICSCONFIG_MEASUREMENTSTEPCONFIG,
    '__module__' : 'motion_metrics_pb2'
    # @@protoc_insertion_point(class_scope:waymo.open_dataset.MotionMetricsConfig.MeasurementStepConfig)
    })
  ,
  'DESCRIPTOR' : _MOTIONMETRICSCONFIG,
  '__module__' : 'motion_metrics_pb2'
  # @@protoc_insertion_point(class_scope:waymo.open_dataset.MotionMetricsConfig)
  })
_sym_db.RegisterMessage(MotionMetricsConfig)
_sym_db.RegisterMessage(MotionMetricsConfig.MeasurementStepConfig)


_SINGLETRAJECTORY.fields_by_name['center_x']._options = None
_SINGLETRAJECTORY.fields_by_name['center_y']._options = None
# @@protoc_insertion_point(module_scope)