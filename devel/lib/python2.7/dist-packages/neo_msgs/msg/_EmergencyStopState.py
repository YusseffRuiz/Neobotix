# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from neo_msgs/EmergencyStopState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class EmergencyStopState(genpy.Message):
  _md5sum = "d857d7312ffc16f75239036504e493e9"
  _type = "neo_msgs/EmergencyStopState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This message holds the emergency stop (EMStop) status of the robot. It detects wether an EMStop is caused by the safety laserscanner or the emergency stop buttons. Moreover, it gives signalizes wether the EMStop was confirmed (after Button press stop) and the system is free again.

# Possible EMStop States
int16 EMFREE = 0 		# system operatign normal
int16 EMSTOP = 1 		# emergency stop is active (Button pressed; obstacle in safety field of scanner)
int16 EMCONFIRMED = 2 		# emergency stop was confirmed system is reinitializing and going back to normal

bool emergency_button_stop	# true = emergency stop signal is issued by button pressed
bool scanner_stop		# true = emergency stop signal is issued by scanner
int16 emergency_state		# state (including confimation by key-switch), values see above

"""
  # Pseudo-constants
  EMFREE = 0
  EMSTOP = 1
  EMCONFIRMED = 2

  __slots__ = ['emergency_button_stop','scanner_stop','emergency_state']
  _slot_types = ['bool','bool','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       emergency_button_stop,scanner_stop,emergency_state

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(EmergencyStopState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.emergency_button_stop is None:
        self.emergency_button_stop = False
      if self.scanner_stop is None:
        self.scanner_stop = False
      if self.emergency_state is None:
        self.emergency_state = 0
    else:
      self.emergency_button_stop = False
      self.scanner_stop = False
      self.emergency_state = 0

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
      buff.write(_get_struct_2Bh().pack(_x.emergency_button_stop, _x.scanner_stop, _x.emergency_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.emergency_button_stop, _x.scanner_stop, _x.emergency_state,) = _get_struct_2Bh().unpack(str[start:end])
      self.emergency_button_stop = bool(self.emergency_button_stop)
      self.scanner_stop = bool(self.scanner_stop)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2Bh().pack(_x.emergency_button_stop, _x.scanner_stop, _x.emergency_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.emergency_button_stop, _x.scanner_stop, _x.emergency_state,) = _get_struct_2Bh().unpack(str[start:end])
      self.emergency_button_stop = bool(self.emergency_button_stop)
      self.scanner_stop = bool(self.scanner_stop)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2Bh = None
def _get_struct_2Bh():
    global _struct_2Bh
    if _struct_2Bh is None:
        _struct_2Bh = struct.Struct("<2Bh")
    return _struct_2Bh
