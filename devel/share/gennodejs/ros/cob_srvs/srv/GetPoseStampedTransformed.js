// Auto-generated. Do not edit!

// (in-package cob_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetPoseStampedTransformedRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target = null;
      this.origin = null;
      this.orientation_override = null;
      this.root_name = null;
      this.tip_name = null;
    }
    else {
      if (initObj.hasOwnProperty('target')) {
        this.target = initObj.target
      }
      else {
        this.target = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('origin')) {
        this.origin = initObj.origin
      }
      else {
        this.origin = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('orientation_override')) {
        this.orientation_override = initObj.orientation_override
      }
      else {
        this.orientation_override = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('root_name')) {
        this.root_name = initObj.root_name
      }
      else {
        this.root_name = '';
      }
      if (initObj.hasOwnProperty('tip_name')) {
        this.tip_name = initObj.tip_name
      }
      else {
        this.tip_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPoseStampedTransformedRequest
    // Serialize message field [target]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.target, buffer, bufferOffset);
    // Serialize message field [origin]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.origin, buffer, bufferOffset);
    // Serialize message field [orientation_override]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.orientation_override, buffer, bufferOffset);
    // Serialize message field [root_name]
    bufferOffset = _serializer.string(obj.root_name, buffer, bufferOffset);
    // Serialize message field [tip_name]
    bufferOffset = _serializer.string(obj.tip_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPoseStampedTransformedRequest
    let len;
    let data = new GetPoseStampedTransformedRequest(null);
    // Deserialize message field [target]
    data.target = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [origin]
    data.origin = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation_override]
    data.orientation_override = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [root_name]
    data.root_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tip_name]
    data.tip_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.target);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.origin);
    length += object.root_name.length;
    length += object.tip_name.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/GetPoseStampedTransformedRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7f6050054c7441aa5c8e1b131c78c448';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PoseStamped target
    geometry_msgs/PoseStamped origin
    geometry_msgs/Quaternion  orientation_override
    string root_name
    string tip_name
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPoseStampedTransformedRequest(null);
    if (msg.target !== undefined) {
      resolved.target = geometry_msgs.msg.PoseStamped.Resolve(msg.target)
    }
    else {
      resolved.target = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.origin !== undefined) {
      resolved.origin = geometry_msgs.msg.PoseStamped.Resolve(msg.origin)
    }
    else {
      resolved.origin = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.orientation_override !== undefined) {
      resolved.orientation_override = geometry_msgs.msg.Quaternion.Resolve(msg.orientation_override)
    }
    else {
      resolved.orientation_override = new geometry_msgs.msg.Quaternion()
    }

    if (msg.root_name !== undefined) {
      resolved.root_name = msg.root_name;
    }
    else {
      resolved.root_name = ''
    }

    if (msg.tip_name !== undefined) {
      resolved.tip_name = msg.tip_name;
    }
    else {
      resolved.tip_name = ''
    }

    return resolved;
    }
};

class GetPoseStampedTransformedResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = new geometry_msgs.msg.PoseStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPoseStampedTransformedResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [result]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPoseStampedTransformedResponse
    let len;
    let data = new GetPoseStampedTransformedResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [result]
    data.result = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.result);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/GetPoseStampedTransformedResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa1ecfaaa2557395e49e336255c7a317';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    geometry_msgs/PoseStamped result
    
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPoseStampedTransformedResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.result !== undefined) {
      resolved.result = geometry_msgs.msg.PoseStamped.Resolve(msg.result)
    }
    else {
      resolved.result = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPoseStampedTransformedRequest,
  Response: GetPoseStampedTransformedResponse,
  md5sum() { return 'ed0069a0a6ce5102fc39c3328eef052f'; },
  datatype() { return 'cob_srvs/GetPoseStampedTransformed'; }
};
