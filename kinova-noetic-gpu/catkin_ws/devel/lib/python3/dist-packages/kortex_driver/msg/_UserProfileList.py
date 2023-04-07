# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/UserProfileList.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class UserProfileList(genpy.Message):
  _md5sum = "2d71d5ab953fb8ddd6c9d9d4a1379bb4"
  _type = "kortex_driver/UserProfileList"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
UserProfile[] user_profiles
================================================================================
MSG: kortex_driver/UserProfile

UserProfileHandle handle
string username
string firstname
string lastname
string application_data
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission"""
  __slots__ = ['user_profiles']
  _slot_types = ['kortex_driver/UserProfile[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       user_profiles

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(UserProfileList, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.user_profiles is None:
        self.user_profiles = []
    else:
      self.user_profiles = []

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
      length = len(self.user_profiles)
      buff.write(_struct_I.pack(length))
      for val1 in self.user_profiles:
        _v1 = val1.handle
        _x = _v1
        buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        _x = val1.username
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.firstname
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.lastname
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.application_data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.user_profiles is None:
        self.user_profiles = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.user_profiles = []
      for i in range(0, length):
        val1 = kortex_driver.msg.UserProfile()
        _v2 = val1.handle
        _x = _v2
        start = end
        end += 8
        (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.username = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.username = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.firstname = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.firstname = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.lastname = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.lastname = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.application_data = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.application_data = str[start:end]
        self.user_profiles.append(val1)
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
      length = len(self.user_profiles)
      buff.write(_struct_I.pack(length))
      for val1 in self.user_profiles:
        _v3 = val1.handle
        _x = _v3
        buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        _x = val1.username
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.firstname
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.lastname
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.application_data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.user_profiles is None:
        self.user_profiles = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.user_profiles = []
      for i in range(0, length):
        val1 = kortex_driver.msg.UserProfile()
        _v4 = val1.handle
        _x = _v4
        start = end
        end += 8
        (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.username = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.username = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.firstname = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.firstname = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.lastname = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.lastname = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.application_data = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.application_data = str[start:end]
        self.user_profiles.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
