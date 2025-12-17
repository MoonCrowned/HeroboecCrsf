from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple, Union


_EPS = 1e-8


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def _clamp01(value: float) -> float:
    return _clamp(value, 0.0, 1.0)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _deg2rad(deg: float) -> float:
    return deg * (math.pi / 180.0)


def _rad2deg(rad: float) -> float:
    return rad * (180.0 / math.pi)


Mat3 = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]


def _mat3_transpose(m: Mat3) -> Mat3:
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def _mat3_mul(a: Mat3, b: Mat3) -> Mat3:
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )


def ConvertQuaternionByBasis(rotation: "Quaternion", basis_matrix: Mat3, inverse_rotation: bool = False) -> "Quaternion":
    r = rotation.ToRotationMatrix()
    if inverse_rotation:
        r = _mat3_transpose(r)
    m = basis_matrix
    mt = _mat3_transpose(m)
    ru = _mat3_mul(_mat3_mul(m, r), mt)
    return Quaternion.FromRotationMatrix(ru)


class classproperty(property):
    def __get__(self, obj, cls):
        return self.fget(cls)  # type: ignore[misc]


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classproperty
    def zero(cls) -> "Vector3":
        return cls(0.0, 0.0, 0.0)

    @classproperty
    def one(cls) -> "Vector3":
        return cls(1.0, 1.0, 1.0)

    @classproperty
    def up(cls) -> "Vector3":
        return cls(0.0, 1.0, 0.0)

    @classproperty
    def down(cls) -> "Vector3":
        return cls(0.0, -1.0, 0.0)

    @classproperty
    def right(cls) -> "Vector3":
        return cls(1.0, 0.0, 0.0)

    @classproperty
    def left(cls) -> "Vector3":
        return cls(-1.0, 0.0, 0.0)

    @classproperty
    def forward(cls) -> "Vector3":
        return cls(0.0, 0.0, 1.0)

    @classproperty
    def back(cls) -> "Vector3":
        return cls(0.0, 0.0, -1.0)

    def copy(self) -> "Vector3":
        return Vector3(self.x, self.y, self.z)

    def Set(self, new_x: float, new_y: float, new_z: float) -> None:
        self.x = float(new_x)
        self.y = float(new_y)
        self.z = float(new_z)

    @property
    def magnitude(self) -> float:
        return math.sqrt(self.sqrMagnitude)

    @property
    def sqrMagnitude(self) -> float:
        return self.x * self.x + self.y * self.y + self.z * self.z

    @property
    def normalized(self) -> "Vector3":
        mag = self.magnitude
        if mag <= _EPS:
            return Vector3.zero
        inv = 1.0 / mag
        return Vector3(self.x * inv, self.y * inv, self.z * inv)

    def Normalize(self) -> None:
        mag = self.magnitude
        if mag <= _EPS:
            self.Set(0.0, 0.0, 0.0)
            return
        inv = 1.0 / mag
        self.x *= inv
        self.y *= inv
        self.z *= inv

    def Scale(self, scale: "Vector3") -> None:
        self.x *= scale.x
        self.y *= scale.y
        self.z *= scale.z

    @staticmethod
    def Dot(lhs: "Vector3", rhs: "Vector3") -> float:
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z

    @staticmethod
    def Cross(lhs: "Vector3", rhs: "Vector3") -> "Vector3":
        return Vector3(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        )

    @staticmethod
    def Distance(a: "Vector3", b: "Vector3") -> float:
        return (a - b).magnitude

    @staticmethod
    def Lerp(a: "Vector3", b: "Vector3", t: float) -> "Vector3":
        t = _clamp01(float(t))
        return Vector3(
            _lerp(a.x, b.x, t),
            _lerp(a.y, b.y, t),
            _lerp(a.z, b.z, t),
        )

    @staticmethod
    def LerpUnclamped(a: "Vector3", b: "Vector3", t: float) -> "Vector3":
        t = float(t)
        return Vector3(
            _lerp(a.x, b.x, t),
            _lerp(a.y, b.y, t),
            _lerp(a.z, b.z, t),
        )

    @staticmethod
    def MoveTowards(current: "Vector3", target: "Vector3", maxDistanceDelta: float) -> "Vector3":
        maxDistanceDelta = float(maxDistanceDelta)
        to_vector = target - current
        dist = to_vector.magnitude
        if dist <= maxDistanceDelta or dist <= _EPS:
            return target.copy()
        return current + to_vector / dist * maxDistanceDelta

    @staticmethod
    def ClampMagnitude(vector: "Vector3", maxLength: float) -> "Vector3":
        maxLength = float(maxLength)
        sqr_mag = vector.sqrMagnitude
        if sqr_mag <= (maxLength * maxLength):
            return vector.copy()
        mag = math.sqrt(sqr_mag)
        if mag <= _EPS:
            return Vector3.zero
        return vector * (maxLength / mag)

    @staticmethod
    def Angle(from_vec: "Vector3", to_vec: "Vector3") -> float:
        denom = math.sqrt(from_vec.sqrMagnitude * to_vec.sqrMagnitude)
        if denom <= _EPS:
            return 0.0
        d = _clamp(Vector3.Dot(from_vec, to_vec) / denom, -1.0, 1.0)
        return _rad2deg(math.acos(d))

    @staticmethod
    def SignedAngle(from_vec: "Vector3", to_vec: "Vector3", axis: "Vector3") -> float:
        unsigned = Vector3.Angle(from_vec, to_vec)
        cross = Vector3.Cross(from_vec, to_vec)
        sign = 1.0 if Vector3.Dot(axis, cross) >= 0.0 else -1.0
        return unsigned * sign

    @staticmethod
    def Project(vector: "Vector3", onNormal: "Vector3") -> "Vector3":
        denom = onNormal.sqrMagnitude
        if denom <= _EPS:
            return Vector3.zero
        return onNormal * (Vector3.Dot(vector, onNormal) / denom)

    @staticmethod
    def ProjectOnPlane(vector: "Vector3", planeNormal: "Vector3") -> "Vector3":
        return vector - Vector3.Project(vector, planeNormal)

    @staticmethod
    def Reflect(inDirection: "Vector3", inNormal: "Vector3") -> "Vector3":
        return inDirection - inNormal * (2.0 * Vector3.Dot(inDirection, inNormal))

    @staticmethod
    def Min(lhs: "Vector3", rhs: "Vector3") -> "Vector3":
        return Vector3(min(lhs.x, rhs.x), min(lhs.y, rhs.y), min(lhs.z, rhs.z))

    @staticmethod
    def Max(lhs: "Vector3", rhs: "Vector3") -> "Vector3":
        return Vector3(max(lhs.x, rhs.x), max(lhs.y, rhs.y), max(lhs.z, rhs.z))

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __neg__(self) -> "Vector3":
        return Vector3(-self.x, -self.y, -self.z)

    def __mul__(self, other: Union[float, int, "Vector3"]) -> "Vector3":
        if isinstance(other, Vector3):
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        s = float(other)
        return Vector3(self.x * s, self.y * s, self.z * s)

    def __rmul__(self, other: Union[float, int, "Vector3"]) -> "Vector3":
        return self.__mul__(other)

    def __truediv__(self, other: Union[float, int]) -> "Vector3":
        s = float(other)
        if abs(s) <= _EPS:
            return Vector3.zero
        inv = 1.0 / s
        return Vector3(self.x * inv, self.y * inv, self.z * inv)

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __repr__(self) -> str:
        return f"Vector3({self.x}, {self.y}, {self.z})"


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classproperty
    def identity(cls) -> "Quaternion":
        return cls(0.0, 0.0, 0.0, 1.0)

    def copy(self) -> "Quaternion":
        return Quaternion(self.x, self.y, self.z, self.w)

    def Set(self, new_x: float, new_y: float, new_z: float, new_w: float) -> None:
        self.x = float(new_x)
        self.y = float(new_y)
        self.z = float(new_z)
        self.w = float(new_w)

    @property
    def sqrMagnitude(self) -> float:
        return self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w

    @property
    def magnitude(self) -> float:
        return math.sqrt(self.sqrMagnitude)

    @property
    def normalized(self) -> "Quaternion":
        mag = self.magnitude
        if mag <= _EPS:
            return Quaternion.identity
        inv = 1.0 / mag
        return Quaternion(self.x * inv, self.y * inv, self.z * inv, self.w * inv)

    def Normalize(self) -> None:
        mag = self.magnitude
        if mag <= _EPS:
            self.Set(0.0, 0.0, 0.0, 1.0)
            return
        inv = 1.0 / mag
        self.x *= inv
        self.y *= inv
        self.z *= inv
        self.w *= inv

    @property
    def conjugate(self) -> "Quaternion":
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def ToRotationMatrix(self) -> Mat3:
        q = self.normalized
        x, y, z, w = q.x, q.y, q.z, q.w

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )

    @staticmethod
    def FromRotationMatrix(m: Mat3) -> "Quaternion":
        return Quaternion._from_rotation_matrix(
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2],
        )

    @staticmethod
    def FromBasis(right: Vector3, up: Vector3, forward: Vector3) -> "Quaternion":
        return Quaternion._from_rotation_matrix(
            right.x, up.x, forward.x,
            right.y, up.y, forward.y,
            right.z, up.z, forward.z,
        )

    @staticmethod
    def Dot(a: "Quaternion", b: "Quaternion") -> float:
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w

    @staticmethod
    def Inverse(rotation: "Quaternion") -> "Quaternion":
        sqr_mag = rotation.sqrMagnitude
        if sqr_mag <= _EPS:
            return Quaternion.identity
        conj = rotation.conjugate
        inv = 1.0 / sqr_mag
        return Quaternion(conj.x * inv, conj.y * inv, conj.z * inv, conj.w * inv)

    @staticmethod
    def Angle(a: "Quaternion", b: "Quaternion") -> float:
        d = abs(Quaternion.Dot(a.normalized, b.normalized))
        d = _clamp(d, -1.0, 1.0)
        return _rad2deg(2.0 * math.acos(d))

    @staticmethod
    def Lerp(a: "Quaternion", b: "Quaternion", t: float) -> "Quaternion":
        t = _clamp01(float(t))
        return Quaternion.LerpUnclamped(a, b, t)

    @staticmethod
    def LerpUnclamped(a: "Quaternion", b: "Quaternion", t: float) -> "Quaternion":
        t = float(t)
        if Quaternion.Dot(a, b) < 0.0:
            b = Quaternion(-b.x, -b.y, -b.z, -b.w)
        q = Quaternion(
            _lerp(a.x, b.x, t),
            _lerp(a.y, b.y, t),
            _lerp(a.z, b.z, t),
            _lerp(a.w, b.w, t),
        )
        return q.normalized

    @staticmethod
    def Slerp(a: "Quaternion", b: "Quaternion", t: float) -> "Quaternion":
        t = _clamp01(float(t))
        return Quaternion.SlerpUnclamped(a, b, t)

    @staticmethod
    def SlerpUnclamped(a: "Quaternion", b: "Quaternion", t: float) -> "Quaternion":
        t = float(t)
        qa = a.normalized
        qb = b.normalized

        dot = Quaternion.Dot(qa, qb)
        if dot < 0.0:
            qb = Quaternion(-qb.x, -qb.y, -qb.z, -qb.w)
            dot = -dot

        if dot > 0.9995:
            return Quaternion.LerpUnclamped(qa, qb, t)

        dot = _clamp(dot, -1.0, 1.0)
        theta_0 = math.acos(dot)
        theta = theta_0 * t
        sin_theta_0 = math.sin(theta_0)
        if abs(sin_theta_0) <= _EPS:
            return qa.copy()

        sin_theta = math.sin(theta)
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return Quaternion(
            qa.x * s0 + qb.x * s1,
            qa.y * s0 + qb.y * s1,
            qa.z * s0 + qb.z * s1,
            qa.w * s0 + qb.w * s1,
        )

    @staticmethod
    def RotateTowards(from_rotation: "Quaternion", to_rotation: "Quaternion", maxDegreesDelta: float) -> "Quaternion":
        angle = Quaternion.Angle(from_rotation, to_rotation)
        if angle <= float(maxDegreesDelta) or angle <= _EPS:
            return to_rotation.copy()
        t = float(maxDegreesDelta) / angle
        return Quaternion.SlerpUnclamped(from_rotation, to_rotation, t)

    @staticmethod
    def AngleAxis(angle_degrees: float, axis: Vector3) -> "Quaternion":
        axis_n = axis.normalized
        if axis_n.sqrMagnitude <= _EPS:
            return Quaternion.identity

        half = _deg2rad(float(angle_degrees)) * 0.5
        s = math.sin(half)
        c = math.cos(half)
        return Quaternion(axis_n.x * s, axis_n.y * s, axis_n.z * s, c)

    @staticmethod
    def Euler(x: Union[float, Vector3], y: Optional[float] = None, z: Optional[float] = None) -> "Quaternion":
        if isinstance(x, Vector3):
            ex = x.x
            ey = x.y
            ez = x.z
        else:
            if y is None or z is None:
                raise TypeError("Quaternion.Euler(x, y, z) requires 3 floats or a Vector3")
            ex = float(x)
            ey = float(y)
            ez = float(z)

        qx = Quaternion.AngleAxis(ex, Vector3.right)
        qy = Quaternion.AngleAxis(ey, Vector3.up)
        qz = Quaternion.AngleAxis(ez, Vector3.forward)

        return qy * qx * qz

    @staticmethod
    def FromToRotation(fromDirection: Vector3, toDirection: Vector3) -> "Quaternion":
        f = fromDirection.normalized
        t = toDirection.normalized
        if f.sqrMagnitude <= _EPS or t.sqrMagnitude <= _EPS:
            return Quaternion.identity

        dot = _clamp(Vector3.Dot(f, t), -1.0, 1.0)
        if dot >= 1.0 - 1e-6:
            return Quaternion.identity
        if dot <= -1.0 + 1e-6:
            axis = Vector3.Cross(Vector3.right, f)
            if axis.sqrMagnitude <= _EPS:
                axis = Vector3.Cross(Vector3.up, f)
            axis = axis.normalized
            return Quaternion.AngleAxis(180.0, axis)

        axis = Vector3.Cross(f, t)
        s = math.sqrt((1.0 + dot) * 2.0)
        invs = 1.0 / s
        return Quaternion(axis.x * invs, axis.y * invs, axis.z * invs, s * 0.5).normalized

    @staticmethod
    def LookRotation(forward: Vector3, upwards: Optional[Vector3] = None) -> "Quaternion":
        f = forward.normalized
        if f.sqrMagnitude <= _EPS:
            return Quaternion.identity

        u = (upwards if upwards is not None else Vector3.up).normalized
        if u.sqrMagnitude <= _EPS:
            u = Vector3.up

        r = Vector3.Cross(u, f)
        if r.sqrMagnitude <= _EPS:
            u = Vector3.forward if abs(f.y) > 0.999 else Vector3.up
            r = Vector3.Cross(u, f)

        r = r.normalized
        u2 = Vector3.Cross(f, r)

        m00 = r.x
        m01 = u2.x
        m02 = f.x
        m10 = r.y
        m11 = u2.y
        m12 = f.y
        m20 = r.z
        m21 = u2.z
        m22 = f.z

        return Quaternion._from_rotation_matrix(m00, m01, m02, m10, m11, m12, m20, m21, m22)

    @staticmethod
    def _from_rotation_matrix(
        m00: float,
        m01: float,
        m02: float,
        m10: float,
        m11: float,
        m12: float,
        m20: float,
        m21: float,
        m22: float,
    ) -> "Quaternion":
        trace = m00 + m11 + m22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (m21 - m12) / s
            y = (m02 - m20) / s
            z = (m10 - m01) / s
            return Quaternion(x, y, z, w).normalized

        if m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
            return Quaternion(x, y, z, w).normalized

        if m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
            return Quaternion(x, y, z, w).normalized

        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
        return Quaternion(x, y, z, w).normalized

    def __mul__(self, other: Union["Quaternion", Vector3]) -> Union["Quaternion", Vector3]:
        if isinstance(other, Quaternion):
            ax, ay, az, aw = self.x, self.y, self.z, self.w
            bx, by, bz, bw = other.x, other.y, other.z, other.w
            return Quaternion(
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            )

        if isinstance(other, Vector3):
            qx, qy, qz, qw = self.x, self.y, self.z, self.w
            vx, vy, vz = other.x, other.y, other.z

            tx = 2.0 * (qy * vz - qz * vy)
            ty = 2.0 * (qz * vx - qx * vz)
            tz = 2.0 * (qx * vy - qy * vx)

            rx = vx + qw * tx + (qy * tz - qz * ty)
            ry = vy + qw * ty + (qz * tx - qx * tz)
            rz = vz + qw * tz + (qx * ty - qy * tx)

            return Vector3(rx, ry, rz)

        return NotImplemented

    def __repr__(self) -> str:
        return f"Quaternion({self.x}, {self.y}, {self.z}, {self.w})"


class Transform:
    __slots__ = ("_position", "_rotation")

    def __init__(
        self,
        position: Optional[Vector3] = None,
        rotation: Optional[Quaternion] = None,
        localScale: Optional[Vector3] = None,
    ):
        self._position = position.copy() if position is not None else Vector3.zero
        self._rotation = rotation.copy() if rotation is not None else Quaternion.identity
        _ = localScale

    @property
    def position(self) -> Vector3:
        return self._position

    @position.setter
    def position(self, value: Vector3) -> None:
        self._position = value.copy()

    @property
    def rotation(self) -> Quaternion:
        return self._rotation

    @rotation.setter
    def rotation(self, value: Quaternion) -> None:
        self._rotation = value.copy()

    @property
    def forward(self) -> Vector3:
        return self._rotation * Vector3.forward

    @property
    def Forward(self) -> Vector3:
        return self.forward

    @property
    def back(self) -> Vector3:
        return self._rotation * Vector3.back

    @property
    def Backward(self) -> Vector3:
        return self.back

    @property
    def right(self) -> Vector3:
        return self._rotation * Vector3.right

    @property
    def Right(self) -> Vector3:
        return self.right

    @property
    def left(self) -> Vector3:
        return self._rotation * Vector3.left

    @property
    def Left(self) -> Vector3:
        return self.left

    @property
    def up(self) -> Vector3:
        return self._rotation * Vector3.up

    @property
    def Up(self) -> Vector3:
        return self.up

    @property
    def down(self) -> Vector3:
        return self._rotation * Vector3.down

    @property
    def Down(self) -> Vector3:
        return self.down

    def TransformVector(self, vector: Vector3) -> Vector3:
        return self._rotation * vector

    def InverseTransformVector(self, vector: Vector3) -> Vector3:
        inv = Quaternion.Inverse(self._rotation)
        return inv * vector

    def TransformPoint(self, point: Vector3) -> Vector3:
        return self._position + (self._rotation * point)

    def InverseTransformPoint(self, point: Vector3) -> Vector3:
        inv = Quaternion.Inverse(self._rotation)
        return inv * (point - self._position)

    def __repr__(self) -> str:
        return f"Transform(position={self._position}, rotation={self._rotation})"
