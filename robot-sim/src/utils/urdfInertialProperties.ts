import { Matrix4, Quaternion, Vector3 } from 'three';
import type { InertialDescription } from '../core/types';

export interface RapierMassProperties {
  mass: number;
  centerOfMass: Vector3;
  principalAngularInertia: Vector3;
  angularInertiaLocalFrame: Quaternion;
}

const principalInertia = (
  tensor: InertialDescription['inertia'],
): { values: [number, number, number]; frame: Quaternion } => {
  const matrix = [
    [tensor.ixx, tensor.ixy, tensor.ixz],
    [tensor.ixy, tensor.iyy, tensor.iyz],
    [tensor.ixz, tensor.iyz, tensor.izz],
  ];
  const vectors = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];

  for (let iteration = 0; iteration < 24; iteration += 1) {
    let p = 0;
    let q = 1;
    for (let row = 0; row < 3; row += 1) {
      for (let column = row + 1; column < 3; column += 1) {
        if (Math.abs(matrix[row][column]) > Math.abs(matrix[p][q])) {
          p = row;
          q = column;
        }
      }
    }
    const offDiagonal = matrix[p][q];
    if (Math.abs(offDiagonal) < 1e-12) break;

    const tau = (matrix[q][q] - matrix[p][p]) / (2 * offDiagonal);
    const tangent = Math.sign(tau || 1) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
    const cosine = 1 / Math.sqrt(1 + tangent * tangent);
    const sine = tangent * cosine;
    const diagonalP = matrix[p][p];
    const diagonalQ = matrix[q][q];
    matrix[p][p] = diagonalP - tangent * offDiagonal;
    matrix[q][q] = diagonalQ + tangent * offDiagonal;
    matrix[p][q] = 0;
    matrix[q][p] = 0;

    for (let index = 0; index < 3; index += 1) {
      if (index !== p && index !== q) {
        const valueP = matrix[index][p];
        const valueQ = matrix[index][q];
        matrix[index][p] = cosine * valueP - sine * valueQ;
        matrix[p][index] = matrix[index][p];
        matrix[index][q] = sine * valueP + cosine * valueQ;
        matrix[q][index] = matrix[index][q];
      }
      const vectorP = vectors[index][p];
      const vectorQ = vectors[index][q];
      vectors[index][p] = cosine * vectorP - sine * vectorQ;
      vectors[index][q] = sine * vectorP + cosine * vectorQ;
    }
  }

  const order = [0, 1, 2].sort((first, second) => matrix[first][first] - matrix[second][second]);
  const firstAxis = new Vector3(
    vectors[0][order[0]],
    vectors[1][order[0]],
    vectors[2][order[0]],
  ).normalize();
  const secondAxis = new Vector3(vectors[0][order[1]], vectors[1][order[1]], vectors[2][order[1]])
    .addScaledVector(
      firstAxis,
      -firstAxis.dot(new Vector3(vectors[0][order[1]], vectors[1][order[1]], vectors[2][order[1]])),
    )
    .normalize();
  const thirdAxis = firstAxis.clone().cross(secondAxis).normalize();
  const frame = new Quaternion().setFromRotationMatrix(
    new Matrix4().makeBasis(firstAxis, secondAxis, thirdAxis),
  );
  const values = order.map((index) => Math.max(matrix[index][index], 1e-9)) as [
    number,
    number,
    number,
  ];

  return { values, frame };
};

export const getRapierMassProperties = (
  inertial: InertialDescription,
  linkToBodyRotation: Quaternion,
): RapierMassProperties => {
  const principal = principalInertia(inertial.inertia);
  const bodyToLinkRotation = linkToBodyRotation.clone().invert();
  const inertialToLinkRotation = new Quaternion(...inertial.origin.orientation);

  return {
    mass: Math.max(inertial.mass, 0.001),
    centerOfMass: new Vector3(...inertial.origin.position).applyQuaternion(bodyToLinkRotation),
    principalAngularInertia: new Vector3(...principal.values),
    angularInertiaLocalFrame: bodyToLinkRotation
      .multiply(inertialToLinkRotation)
      .multiply(principal.frame),
  };
};
