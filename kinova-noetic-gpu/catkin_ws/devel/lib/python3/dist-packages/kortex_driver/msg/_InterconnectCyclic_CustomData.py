# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/InterconnectCyclic_CustomData.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class InterconnectCyclic_CustomData(genpy.Message):
  _md5sum = "4f180448c3bd7285547e1bae08efc80a"
  _type = "kortex_driver/InterconnectCyclic_CustomData"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
InterconnectCyclic_MessageId custom_data_id
uint32 custom_data_0
uint32 custom_data_1
uint32 custom_data_2
uint32 custom_data_3
uint32 custom_data_4
uint32 custom_data_5
uint32 custom_data_6
uint32 custom_data_7
uint32 custom_data_8
uint32 custom_data_9
uint32 custom_data_10
uint32 custom_data_11
uint32 custom_data_12
uint32 custom_data_13
uint32 custom_data_14
uint32 custom_data_15
InterconnectCyclic_CustomData_tool_customData oneof_tool_customData
================================================================================
MSG: kortex_driver/InterconnectCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/InterconnectCyclic_CustomData_tool_customData

GripperCyclic_CustomData[] gripper_custom_data
================================================================================
MSG: kortex_driver/GripperCyclic_CustomData

GripperCyclic_MessageId custom_data_id
CustomDataUnit gripper_custom_data
CustomDataUnit[] motor_custom_data
================================================================================
MSG: kortex_driver/GripperCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/CustomDataUnit

uint32 custom_data_0
uint32 custom_data_1
uint32 custom_data_2
uint32 custom_data_3
uint32 custom_data_4
uint32 custom_data_5
uint32 custom_data_6
uint32 custom_data_7
uint32 custom_data_8
uint32 custom_data_9
uint32 custom_data_10
uint32 custom_data_11
uint32 custom_data_12
uint32 custom_data_13
uint32 custom_data_14
uint32 custom_data_15"""
  __slots__ = ['custom_data_id','custom_data_0','custom_data_1','custom_data_2','custom_data_3','custom_data_4','custom_data_5','custom_data_6','custom_data_7','custom_data_8','custom_data_9','custom_data_10','custom_data_11','custom_data_12','custom_data_13','custom_data_14','custom_data_15','oneof_tool_customData']
  _slot_types = ['kortex_driver/InterconnectCyclic_MessageId','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','uint32','kortex_driver/InterconnectCyclic_CustomData_tool_customData']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       custom_data_id,custom_data_0,custom_data_1,custom_data_2,custom_data_3,custom_data_4,custom_data_5,custom_data_6,custom_data_7,custom_data_8,custom_data_9,custom_data_10,custom_data_11,custom_data_12,custom_data_13,custom_data_14,custom_data_15,oneof_tool_customData

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(InterconnectCyclic_CustomData, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.custom_data_id is None:
        self.custom_data_id = kortex_driver.msg.InterconnectCyclic_MessageId()
      if self.custom_data_0 is None:
        self.custom_data_0 = 0
      if self.custom_data_1 is None:
        self.custom_data_1 = 0
      if self.custom_data_2 is None:
        self.custom_data_2 = 0
      if self.custom_data_3 is None:
        self.custom_data_3 = 0
      if self.custom_data_4 is None:
        self.custom_data_4 = 0
      if self.custom_data_5 is None:
        self.custom_data_5 = 0
      if self.custom_data_6 is None:
        self.custom_data_6 = 0
      if self.custom_data_7 is None:
        self.custom_data_7 = 0
      if self.custom_data_8 is None:
        self.custom_data_8 = 0
      if self.custom_data_9 is None:
        self.custom_data_9 = 0
      if self.custom_data_10 is None:
        self.custom_data_10 = 0
      if self.custom_data_11 is None:
        self.custom_data_11 = 0
      if self.custom_data_12 is None:
        self.custom_data_12 = 0
      if self.custom_data_13 is None:
        self.custom_data_13 = 0
      if self.custom_data_14 is None:
        self.custom_data_14 = 0
      if self.custom_data_15 is None:
        self.custom_data_15 = 0
      if self.oneof_tool_customData is None:
        self.oneof_tool_customData = kortex_driver.msg.InterconnectCyclic_CustomData_tool_customData()
    else:
      self.custom_data_id = kortex_driver.msg.InterconnectCyclic_MessageId()
      self.custom_data_0 = 0
      self.custom_data_1 = 0
      self.custom_data_2 = 0
      self.custom_data_3 = 0
      self.custom_data_4 = 0
      self.custom_data_5 = 0
      self.custom_data_6 = 0
      self.custom_data_7 = 0
      self.custom_data_8 = 0
      self.custom_data_9 = 0
      self.custom_data_10 = 0
      self.custom_data_11 = 0
      self.custom_data_12 = 0
      self.custom_data_13 = 0
      self.custom_data_14 = 0
      self.custom_data_15 = 0
      self.oneof_tool_customData = kortex_driver.msg.InterconnectCyclic_CustomData_tool_customData()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_17I().pack(_x.custom_data_id.identifier, _x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15))
      length = len(self.oneof_tool_customData.gripper_custom_data)
      buff.write(_struct_I.pack(length))
      for val1 in self.oneof_tool_customData.gripper_custom_data:
        _v1 = val1.custom_data_id
        _x = _v1.identifier
        buff.write(_get_struct_I().pack(_x))
        _v2 = val1.gripper_custom_data
        _x = _v2
        buff.write(_get_struct_16I().pack(_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15))
        length = len(val1.motor_custom_data)
        buff.write(_struct_I.pack(length))
        for val2 in val1.motor_custom_data:
          _x = val2
          buff.write(_get_struct_16I().pack(_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.custom_data_id is None:
        self.custom_data_id = kortex_driver.msg.InterconnectCyclic_MessageId()
      if self.oneof_tool_customData is None:
        self.oneof_tool_customData = kortex_driver.msg.InterconnectCyclic_CustomData_tool_customData()
      end = 0
      _x = self
      start = end
      end += 68
      (_x.custom_data_id.identifier, _x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15,) = _get_struct_17I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.oneof_tool_customData.gripper_custom_data = []
      for i in range(0, length):
        val1 = kortex_driver.msg.GripperCyclic_CustomData()
        _v3 = val1.custom_data_id
        start = end
        end += 4
        (_v3.identifier,) = _get_struct_I().unpack(str[start:end])
        _v4 = val1.gripper_custom_data
        _x = _v4
        start = end
        end += 64
        (_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15,) = _get_struct_16I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.motor_custom_data = []
        for i in range(0, length):
          val2 = kortex_driver.msg.CustomDataUnit()
          _x = val2
          start = end
          end += 64
          (_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15,) = _get_struct_16I().unpack(str[start:end])
          val1.motor_custom_data.append(val2)
        self.oneof_tool_customData.gripper_custom_data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_17I().pack(_x.custom_data_id.identifier, _x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15))
      length = len(self.oneof_tool_customData.gripper_custom_data)
      buff.write(_struct_I.pack(length))
      for val1 in self.oneof_tool_customData.gripper_custom_data:
        _v5 = val1.custom_data_id
        _x = _v5.identifier
        buff.write(_get_struct_I().pack(_x))
        _v6 = val1.gripper_custom_data
        _x = _v6
        buff.write(_get_struct_16I().pack(_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15))
        length = len(val1.motor_custom_data)
        buff.write(_struct_I.pack(length))
        for val2 in val1.motor_custom_data:
          _x = val2
          buff.write(_get_struct_16I().pack(_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.custom_data_id is None:
        self.custom_data_id = kortex_driver.msg.InterconnectCyclic_MessageId()
      if self.oneof_tool_customData is None:
        self.oneof_tool_customData = kortex_driver.msg.InterconnectCyclic_CustomData_tool_customData()
      end = 0
      _x = self
      start = end
      end += 68
      (_x.custom_data_id.identifier, _x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15,) = _get_struct_17I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.oneof_tool_customData.gripper_custom_data = []
      for i in range(0, length):
        val1 = kortex_driver.msg.GripperCyclic_CustomData()
        _v7 = val1.custom_data_id
        start = end
        end += 4
        (_v7.identifier,) = _get_struct_I().unpack(str[start:end])
        _v8 = val1.gripper_custom_data
        _x = _v8
        start = end
        end += 64
        (_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15,) = _get_struct_16I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.motor_custom_data = []
        for i in range(0, length):
          val2 = kortex_driver.msg.CustomDataUnit()
          _x = val2
          start = end
          end += 64
          (_x.custom_data_0, _x.custom_data_1, _x.custom_data_2, _x.custom_data_3, _x.custom_data_4, _x.custom_data_5, _x.custom_data_6, _x.custom_data_7, _x.custom_data_8, _x.custom_data_9, _x.custom_data_10, _x.custom_data_11, _x.custom_data_12, _x.custom_data_13, _x.custom_data_14, _x.custom_data_15,) = _get_struct_16I().unpack(str[start:end])
          val1.motor_custom_data.append(val2)
        self.oneof_tool_customData.gripper_custom_data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_16I = None
def _get_struct_16I():
    global _struct_16I
    if _struct_16I is None:
        _struct_16I = struct.Struct("<16I")
    return _struct_16I
_struct_17I = None
def _get_struct_17I():
    global _struct_17I
    if _struct_17I is None:
        _struct_17I = struct.Struct("<17I")
    return _struct_17I
