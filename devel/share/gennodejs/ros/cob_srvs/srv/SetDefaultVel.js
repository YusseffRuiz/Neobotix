// Auto-generated. Do not edit!

// (in-package cob_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SetDefaultVelRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.default_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('default_vel')) {
        this.default_vel = initObj.default_vel
      }
      else {
        this.default_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDefaultVelRequest
    // Serialize message field [default_vel]
    bufferOffset = _serializer.float32(obj.default_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDefaultVelRequest
    let len;
    let data = new SetDefaultVelRequest(null);
    // Deserialize message field [default_vel]
    data.default_vel = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/SetDefaultVelRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06408962e6bcadda2e07880ad9dedbec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 default_vel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDefaultVelRequest(null);
    if (msg.default_vel !== undefined) {
      resolved.default_vel = msg.default_vel;
    }
    else {
      resolved.default_vel = 0.0
    }

    return resolved;
    }
};

class SetDefaultVelResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error_message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = new std_msgs.msg.Bool();
      }
      if (initObj.hasOwnProperty('error_message')) {
        this.error_message = initObj.error_message
      }
      else {
        this.error_message = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDefaultVelResponse
    // Serialize message field [success]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.success, buffer, bufferOffset);
    // Serialize message field [error_message]
    bufferOffset = std_msgs.msg.String.serialize(obj.error_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDefaultVelResponse
    let len;
    let data = new SetDefaultVelResponse(null);
    // Deserialize message field [success]
    data.success = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    // Deserialize message field [error_message]
    data.error_message = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.error_message);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/SetDefaultVelResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bdad515e8f4c013bd650d3407753d2b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Bool success
    std_msgs/String error_message
    
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDefaultVelResponse(null);
    if (msg.success !== undefined) {
      resolved.success = std_msgs.msg.Bool.Resolve(msg.success)
    }
    else {
      resolved.success = new std_msgs.msg.Bool()
    }

    if (msg.error_message !== undefined) {
      resolved.error_message = std_msgs.msg.String.Resolve(msg.error_message)
    }
    else {
      resolved.error_message = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetDefaultVelRequest,
  Response: SetDefaultVelResponse,
  md5sum() { return 'a7159156625f755343070ad39bfaf3a3'; },
  datatype() { return 'cob_srvs/SetDefaultVel'; }
};
