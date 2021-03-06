"""autogenerated by genmsg_py from Marker.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg

class Marker(roslib.message.Message):
  _md5sum = "da6f93747c24b19a71932ae4b040f489"
  _type = "vicon_bridge/Marker"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string marker_name
string subject_name
string segment_name
geometry_msgs/Point translation
bool occluded

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

"""
  __slots__ = ['marker_name','subject_name','segment_name','translation','occluded']
  _slot_types = ['string','string','string','geometry_msgs/Point','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       marker_name,subject_name,segment_name,translation,occluded
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Marker, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.marker_name is None:
        self.marker_name = ''
      if self.subject_name is None:
        self.subject_name = ''
      if self.segment_name is None:
        self.segment_name = ''
      if self.translation is None:
        self.translation = geometry_msgs.msg.Point()
      if self.occluded is None:
        self.occluded = False
    else:
      self.marker_name = ''
      self.subject_name = ''
      self.segment_name = ''
      self.translation = geometry_msgs.msg.Point()
      self.occluded = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self.marker_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.subject_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.segment_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3dB.pack(_x.translation.x, _x.translation.y, _x.translation.z, _x.occluded))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.translation is None:
        self.translation = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.marker_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.subject_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.segment_name = str[start:end]
      _x = self
      start = end
      end += 25
      (_x.translation.x, _x.translation.y, _x.translation.z, _x.occluded,) = _struct_3dB.unpack(str[start:end])
      self.occluded = bool(self.occluded)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self.marker_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.subject_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.segment_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3dB.pack(_x.translation.x, _x.translation.y, _x.translation.z, _x.occluded))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.translation is None:
        self.translation = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.marker_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.subject_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.segment_name = str[start:end]
      _x = self
      start = end
      end += 25
      (_x.translation.x, _x.translation.y, _x.translation.z, _x.occluded,) = _struct_3dB.unpack(str[start:end])
      self.occluded = bool(self.occluded)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3dB = struct.Struct("<3dB")
