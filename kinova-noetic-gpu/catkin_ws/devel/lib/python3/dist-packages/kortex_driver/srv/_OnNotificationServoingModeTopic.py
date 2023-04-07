# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/OnNotificationServoingModeTopicRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class OnNotificationServoingModeTopicRequest(genpy.Message):
  _md5sum = "942d7c40a40678d0347656d7bafd96e9"
  _type = "kortex_driver/OnNotificationServoingModeTopicRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """NotificationOptions input

================================================================================
MSG: kortex_driver/NotificationOptions

uint32 type
uint32 rate_m_sec
float32 threshold_value"""
  __slots__ = ['input']
  _slot_types = ['kortex_driver/NotificationOptions']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       input

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OnNotificationServoingModeTopicRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.input is None:
        self.input = kortex_driver.msg.NotificationOptions()
    else:
      self.input = kortex_driver.msg.NotificationOptions()

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
      buff.write(_get_struct_2If().pack(_x.input.type, _x.input.rate_m_sec, _x.input.threshold_value))
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
      if self.input is None:
        self.input = kortex_driver.msg.NotificationOptions()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.input.type, _x.input.rate_m_sec, _x.input.threshold_value,) = _get_struct_2If().unpack(str[start:end])
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
      buff.write(_get_struct_2If().pack(_x.input.type, _x.input.rate_m_sec, _x.input.threshold_value))
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
      if self.input is None:
        self.input = kortex_driver.msg.NotificationOptions()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.input.type, _x.input.rate_m_sec, _x.input.threshold_value,) = _get_struct_2If().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2If = None
def _get_struct_2If():
    global _struct_2If
    if _struct_2If is None:
        _struct_2If = struct.Struct("<2If")
    return _struct_2If
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/OnNotificationServoingModeTopicResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class OnNotificationServoingModeTopicResponse(genpy.Message):
  _md5sum = "29ff9348c5c8343d487a90668267a29e"
  _type = "kortex_driver/OnNotificationServoingModeTopicResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """NotificationHandle output

================================================================================
MSG: kortex_driver/NotificationHandle

uint32 identifier"""
  __slots__ = ['output']
  _slot_types = ['kortex_driver/NotificationHandle']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       output

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OnNotificationServoingModeTopicResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.output is None:
        self.output = kortex_driver.msg.NotificationHandle()
    else:
      self.output = kortex_driver.msg.NotificationHandle()

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
      _x = self.output.identifier
      buff.write(_get_struct_I().pack(_x))
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
      if self.output is None:
        self.output = kortex_driver.msg.NotificationHandle()
      end = 0
      start = end
      end += 4
      (self.output.identifier,) = _get_struct_I().unpack(str[start:end])
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
      _x = self.output.identifier
      buff.write(_get_struct_I().pack(_x))
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
      if self.output is None:
        self.output = kortex_driver.msg.NotificationHandle()
      end = 0
      start = end
      end += 4
      (self.output.identifier,) = _get_struct_I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
class OnNotificationServoingModeTopic(object):
  _type          = 'kortex_driver/OnNotificationServoingModeTopic'
  _md5sum = '6fefdd07c6cb63a94f7b48e7e07e815b'
  _request_class  = OnNotificationServoingModeTopicRequest
  _response_class = OnNotificationServoingModeTopicResponse
