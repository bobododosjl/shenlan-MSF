// Auto-generated. Do not edit!

// (in-package lidar_localization.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class saveMapRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type saveMapRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type saveMapRequest
    let len;
    let data = new saveMapRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lidar_localization/saveMapRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new saveMapRequest(null);
    return resolved;
    }
};

class saveMapResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.succeed = null;
    }
    else {
      if (initObj.hasOwnProperty('succeed')) {
        this.succeed = initObj.succeed
      }
      else {
        this.succeed = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type saveMapResponse
    // Serialize message field [succeed]
    bufferOffset = _serializer.bool(obj.succeed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type saveMapResponse
    let len;
    let data = new saveMapResponse(null);
    // Deserialize message field [succeed]
    data.succeed = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lidar_localization/saveMapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8d9c3b918a0afafe09791ef8d7853918';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool succeed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new saveMapResponse(null);
    if (msg.succeed !== undefined) {
      resolved.succeed = msg.succeed;
    }
    else {
      resolved.succeed = false
    }

    return resolved;
    }
};

module.exports = {
  Request: saveMapRequest,
  Response: saveMapResponse,
  md5sum() { return '8d9c3b918a0afafe09791ef8d7853918'; },
  datatype() { return 'lidar_localization/saveMap'; }
};
