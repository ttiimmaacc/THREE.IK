import { Matrix4, Vector3, Object3D, Quaternion } from 'three';
import { transformPoint, getCentroid, getWorldPosition, setQuaternionFromDirection } from './utils.js';
import IKBallConstraint from './IKBallConstraint.js';

const Y_AXIS = new Vector3(0, 1, 0);

/**
 * A class for a joint.
 */
export class IKJoint extends Object3D {
  /**
   * @param {THREE.Bone} bone
   * @param {Object} config
   * @param {Array<IKConstraint>} [config.constraints]
   */
  constructor(bone, { constraints } = {}) {
    super();
    this.constraints = constraints || [];
    this.bone = bone;
    this.distance = 0;
    this._originalDirection = new Vector3();
    this._direction = new Vector3();
    this._worldPosition = new Vector3();
    this._isSubBase = false;
    this._subBasePositions = null;
    this.isIKJoint = true;
    this._tempQuaternion = new Quaternion();
    this._updateWorldPosition();

    // Ensure the IKJoint follows the bone's transformations
    this.bone.add(this);
  }

  /**
   * @private
   */
  _setIsSubBase() {
    this._isSubBase = true;
    this._subBasePositions = [];
  }

  /**
   * Consumes the stored sub base positions and apply it as this
   * joint's world position, clearing the sub base positions.
   *
   * @private
   */
  _applySubBasePositions() {
    if (this._subBasePositions.length === 0) {
      return;
    }
    getCentroid(this._subBasePositions, this._worldPosition);
    this._subBasePositions.length = 0;
  }

  /**
   * @private
   */
  _applyConstraints() {
    if (!this.constraints) {
      return;
    }

    let constraintApplied = false;
    for (const constraint of this.constraints) {
      if (constraint && constraint._apply) {
        const applied = constraint._apply(this);
        constraintApplied = constraintApplied || applied;
      }
    }
    return constraintApplied;
  }

  /**
   * Set the distance.
   * @private
   * @param {number} distance
   */
  _setDistance(distance) {
    this.distance = distance;
  }

  /**
   * @private
   */
  _getDirection() {
    return this._direction;
  }

  /**
   * @private
   */
  _setDirection(direction) {
    this._direction.copy(direction);
  }

  /**
   * Gets the distance.
   * @private
   * @return {number}
   */
  _getDistance() {
    return this.distance;
  }

  /**
   * @private
   */
  _updateMatrixWorld() {
    this.bone.updateWorldMatrix(true, false);
  }

  /**
   * @private
   * @return {THREE.Vector3}
   */
  _getWorldPosition() {
    return this._worldPosition;
  }

  /**
   * @private
   */
  _getWorldDirection(joint) {
    return new Vector3().subVectors(this._getWorldPosition(), joint._getWorldPosition()).normalize();
  }

  /**
   * @private
   */
  _updateWorldPosition() {
    getWorldPosition(this.bone, this._worldPosition);
  }

  /**
   * @private
   */
  _setWorldPosition(position) {
    this._worldPosition.copy(position);
  }

  /**
   * @private
   */
  _localToWorldDirection(direction) {
    if (this.bone.parent) {
      direction.transformDirection(this.bone.parent.matrixWorld);
    }
    return direction;
  }

  /**
   * @private
   */
  _worldToLocalDirection(direction) {
    if (this.bone.parent) {
      direction.transformDirection(new Matrix4().copy(this.bone.parent.matrixWorld).invert());
    }
    return direction;
  }

  /**
   * @private
   */
  _applyWorldPosition() {
    const direction = new Vector3().copy(this._direction);
    const position = new Vector3().copy(this._getWorldPosition());

    const parent = this.bone.parent;

    if (parent) {
      this._updateMatrixWorld();
      // Transform position into the parent's local space
      position.applyMatrix4(new Matrix4().copy(parent.matrixWorld).invert());
      this.bone.position.copy(position);

      this._updateMatrixWorld();

      // Transform direction into local space
      direction.transformDirection(new Matrix4().copy(parent.matrixWorld).invert());
      setQuaternionFromDirection(direction, Y_AXIS, this._tempQuaternion);
      this.bone.quaternion.copy(this._tempQuaternion);
    } else {
      this.bone.position.copy(position);
    }

    // Update the world matrix so the next joint can properly transform
    // with this world matrix
    this.bone.updateMatrix();
    this.bone.updateWorldMatrix(true, false);
    this._updateWorldPosition();
  }

  /**
   * @param {IKJoint|THREE.Vector3}
   * @private
   * @return {number}
   */
  _getWorldDistance(joint) {
    return this._worldPosition.distanceTo(joint.isIKJoint ? joint._getWorldPosition() : getWorldPosition(joint, new Vector3()));
  }
}
