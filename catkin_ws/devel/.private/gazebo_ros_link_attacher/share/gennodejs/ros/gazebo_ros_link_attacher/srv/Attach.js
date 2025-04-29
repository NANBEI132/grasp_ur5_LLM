// Auto-generated. Do not edit!

// (in-package gazebo_ros_link_attacher.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class AttachRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model_name_1 = null;
      this.link_name_1 = null;
      this.model_name_2 = null;
      this.link_name_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('model_name_1')) {
        this.model_name_1 = initObj.model_name_1
      }
      else {
        this.model_name_1 = '';
      }
      if (initObj.hasOwnProperty('link_name_1')) {
        this.link_name_1 = initObj.link_name_1
      }
      else {
        this.link_name_1 = '';
      }
      if (initObj.hasOwnProperty('model_name_2')) {
        this.model_name_2 = initObj.model_name_2
      }
      else {
        this.model_name_2 = '';
      }
      if (initObj.hasOwnProperty('link_name_2')) {
        this.link_name_2 = initObj.link_name_2
      }
      else {
        this.link_name_2 = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AttachRequest
    // Serialize message field [model_name_1]
    bufferOffset = _serializer.string(obj.model_name_1, buffer, bufferOffset);
    // Serialize message field [link_name_1]
    bufferOffset = _serializer.string(obj.link_name_1, buffer, bufferOffset);
    // Serialize message field [model_name_2]
    bufferOffset = _serializer.string(obj.model_name_2, buffer, bufferOffset);
    // Serialize message field [link_name_2]
    bufferOffset = _serializer.string(obj.link_name_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AttachRequest
    let len;
    let data = new AttachRequest(null);
    // Deserialize message field [model_name_1]
    data.model_name_1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [link_name_1]
    data.link_name_1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [model_name_2]
    data.model_name_2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [link_name_2]
    data.link_name_2 = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.model_name_1);
    length += _getByteLength(object.link_name_1);
    length += _getByteLength(object.model_name_2);
    length += _getByteLength(object.link_name_2);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'gazebo_ros_link_attacher/AttachRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ff39d0bc8e054b10e21a2f298cb7fb05';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string model_name_1
    string link_name_1
    string model_name_2
    string link_name_2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AttachRequest(null);
    if (msg.model_name_1 !== undefined) {
      resolved.model_name_1 = msg.model_name_1;
    }
    else {
      resolved.model_name_1 = ''
    }

    if (msg.link_name_1 !== undefined) {
      resolved.link_name_1 = msg.link_name_1;
    }
    else {
      resolved.link_name_1 = ''
    }

    if (msg.model_name_2 !== undefined) {
      resolved.model_name_2 = msg.model_name_2;
    }
    else {
      resolved.model_name_2 = ''
    }

    if (msg.link_name_2 !== undefined) {
      resolved.link_name_2 = msg.link_name_2;
    }
    else {
      resolved.link_name_2 = ''
    }

    return resolved;
    }
};

class AttachResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AttachResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AttachResponse
    let len;
    let data = new AttachResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'gazebo_ros_link_attacher/AttachResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f6da3883749771fac40d6deb24a8c02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ok
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AttachResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
    }

    return resolved;
    }
};

module.exports = {
  Request: AttachRequest,
  Response: AttachResponse,
  md5sum() { return 'c91fb3be70ce66d19130d40294cf4bd5'; },
  datatype() { return 'gazebo_ros_link_attacher/Attach'; }
};
