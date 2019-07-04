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

class SetJointStiffnessRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_stiffness = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_stiffness')) {
        this.joint_stiffness = initObj.joint_stiffness
      }
      else {
        this.joint_stiffness = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointStiffnessRequest
    // Serialize message field [joint_stiffness]
    bufferOffset = _arraySerializer.float32(obj.joint_stiffness, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointStiffnessRequest
    let len;
    let data = new SetJointStiffnessRequest(null);
    // Deserialize message field [joint_stiffness]
    data.joint_stiffness = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.joint_stiffness.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/SetJointStiffnessRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3f30c4d99702f028fe1bef65668a3d21';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] joint_stiffness
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointStiffnessRequest(null);
    if (msg.joint_stiffness !== undefined) {
      resolved.joint_stiffness = msg.joint_stiffness;
    }
    else {
      resolved.joint_stiffness = []
    }

    return resolved;
    }
};

class SetJointStiffnessResponse {
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
    // Serializes a message object of type SetJointStiffnessResponse
    // Serialize message field [success]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.success, buffer, bufferOffset);
    // Serialize message field [error_message]
    bufferOffset = std_msgs.msg.String.serialize(obj.error_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointStiffnessResponse
    let len;
    let data = new SetJointStiffnessResponse(null);
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
    return 'cob_srvs/SetJointStiffnessResponse';
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
    const resolved = new SetJointStiffnessResponse(null);
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
  Request: SetJointStiffnessRequest,
  Response: SetJointStiffnessResponse,
  md5sum() { return '2ae9ddd5d3294ccf3c876d966e4e11a3'; },
  datatype() { return 'cob_srvs/SetJointStiffness'; }
};
