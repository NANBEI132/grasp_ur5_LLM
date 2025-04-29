// Auto-generated. Do not edit!

// (in-package vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Lego_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.posx = null;
      this.posy = null;
      this.posz = null;
      this.rotx = null;
      this.roty = null;
      this.rotz = null;
      this.class_id = null;
    }
    else {
      if (initObj.hasOwnProperty('posx')) {
        this.posx = initObj.posx
      }
      else {
        this.posx = 0.0;
      }
      if (initObj.hasOwnProperty('posy')) {
        this.posy = initObj.posy
      }
      else {
        this.posy = 0.0;
      }
      if (initObj.hasOwnProperty('posz')) {
        this.posz = initObj.posz
      }
      else {
        this.posz = 0.0;
      }
      if (initObj.hasOwnProperty('rotx')) {
        this.rotx = initObj.rotx
      }
      else {
        this.rotx = 0.0;
      }
      if (initObj.hasOwnProperty('roty')) {
        this.roty = initObj.roty
      }
      else {
        this.roty = 0.0;
      }
      if (initObj.hasOwnProperty('rotz')) {
        this.rotz = initObj.rotz
      }
      else {
        this.rotz = 0.0;
      }
      if (initObj.hasOwnProperty('class_id')) {
        this.class_id = initObj.class_id
      }
      else {
        this.class_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lego_state
    // Serialize message field [posx]
    bufferOffset = _serializer.float64(obj.posx, buffer, bufferOffset);
    // Serialize message field [posy]
    bufferOffset = _serializer.float64(obj.posy, buffer, bufferOffset);
    // Serialize message field [posz]
    bufferOffset = _serializer.float64(obj.posz, buffer, bufferOffset);
    // Serialize message field [rotx]
    bufferOffset = _serializer.float64(obj.rotx, buffer, bufferOffset);
    // Serialize message field [roty]
    bufferOffset = _serializer.float64(obj.roty, buffer, bufferOffset);
    // Serialize message field [rotz]
    bufferOffset = _serializer.float64(obj.rotz, buffer, bufferOffset);
    // Serialize message field [class_id]
    bufferOffset = _serializer.int64(obj.class_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lego_state
    let len;
    let data = new Lego_state(null);
    // Deserialize message field [posx]
    data.posx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [posy]
    data.posy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [posz]
    data.posz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rotx]
    data.rotx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roty]
    data.roty = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rotz]
    data.rotz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [class_id]
    data.class_id = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/Lego_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b52ed836370681e6ac7dd0d073d7baf5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 posx
    float64 posy
    float64 posz
    float64 rotx
    float64 roty
    float64 rotz
    int64 class_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lego_state(null);
    if (msg.posx !== undefined) {
      resolved.posx = msg.posx;
    }
    else {
      resolved.posx = 0.0
    }

    if (msg.posy !== undefined) {
      resolved.posy = msg.posy;
    }
    else {
      resolved.posy = 0.0
    }

    if (msg.posz !== undefined) {
      resolved.posz = msg.posz;
    }
    else {
      resolved.posz = 0.0
    }

    if (msg.rotx !== undefined) {
      resolved.rotx = msg.rotx;
    }
    else {
      resolved.rotx = 0.0
    }

    if (msg.roty !== undefined) {
      resolved.roty = msg.roty;
    }
    else {
      resolved.roty = 0.0
    }

    if (msg.rotz !== undefined) {
      resolved.rotz = msg.rotz;
    }
    else {
      resolved.rotz = 0.0
    }

    if (msg.class_id !== undefined) {
      resolved.class_id = msg.class_id;
    }
    else {
      resolved.class_id = 0
    }

    return resolved;
    }
};

module.exports = Lego_state;
