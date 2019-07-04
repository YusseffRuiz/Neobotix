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

class SetMaxVelRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.max_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('max_vel')) {
        this.max_vel = initObj.max_vel
      }
      else {
        this.max_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMaxVelRequest
    // Serialize message field [max_vel]
    bufferOffset = _serializer.float32(obj.max_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMaxVelRequest
    let len;
    let data = new SetMaxVelRequest(null);
    // Deserialize message field [max_vel]
    data.max_vel = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/SetMaxVelRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd4c36ffe76b55c7212cbc93d9e269be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 max_vel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetMaxVelRequest(null);
    if (msg.max_vel !== undefined) {
      resolved.max_vel = msg.max_vel;
    }
    else {
      resolved.max_vel = 0.0
    }

    return resolved;
    }
};

class SetMaxVelResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.errorMessage = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('errorMessage')) {
        this.errorMessage = initObj.errorMessage
      }
      else {
        this.errorMessage = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMaxVelResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [errorMessage]
    bufferOffset = std_msgs.msg.String.serialize(obj.errorMessage, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMaxVelResponse
    let len;
    let data = new SetMaxVelResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [errorMessage]
    data.errorMessage = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.errorMessage);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/SetMaxVelResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c3b62f691308002270bd6702fd7da94b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    std_msgs/String errorMessage
    
    
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
    const resolved = new SetMaxVelResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.errorMessage !== undefined) {
      resolved.errorMessage = std_msgs.msg.String.Resolve(msg.errorMessage)
    }
    else {
      resolved.errorMessage = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetMaxVelRequest,
  Response: SetMaxVelResponse,
  md5sum() { return '5212ea5f4e683819f4e395ad0eb8ae8c'; },
  datatype() { return 'cob_srvs/SetMaxVel'; }
};
