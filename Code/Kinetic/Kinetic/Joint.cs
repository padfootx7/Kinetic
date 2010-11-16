using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Kinetic
{
    public class Joint
    {
        // Joint variables
        public Vector2 LocalAnchor1, LocalAnchor2;
        public Vector2 R1, R2;
        public Vector2 Bias;
        public Vector2 AccumulatedImpulses;

        public Body Body1, Body2;

        public float BiasFactor;
        public float Softness;

        public Matrix2 M;

        public Joint(Body body1, Body body2, Vector2 anchor)
        {
            Body1 = body1;
            Body2 = body2;

            Matrix2 rotation1 = new Matrix2(body1.Rotation);
            Matrix2 rotation2 = new Matrix2(body2.Rotation);

            Matrix2 rotation1T = rotation1.Transpose();
            Matrix2 rotation2T = rotation2.Transpose();

            LocalAnchor1 = Matrix2.Multiply(rotation1T, Vector2.Subtract(anchor, body1.Position));
            LocalAnchor2 = Matrix2.Multiply(rotation2T, Vector2.Subtract(anchor, body2.Position));

            // Set specific variables
            BiasFactor = 0.2f;
        }

        public void Prestep(float inverseDt)
        {
            Matrix2 rotation1 = new Matrix2(Body1.Rotation);
            Matrix2 rotation2 = new Matrix2(Body2.Rotation);

            R1 = Matrix2.Multiply(rotation1, LocalAnchor1);
            R2 = Matrix2.Multiply(rotation2, LocalAnchor2);

            // Incoming loads of math
            Matrix2 k1 = new Matrix2
            (
                Body1.InverseMass + Body2.InverseMass, 0f, 
                0f, Body1.InverseMass + Body2.InverseMass
            );

            Matrix2 k2 = new Matrix2
            (
                Body1.InverseInertia * R1.Y * R1.Y, -Body1.InverseInertia * R1.X * R1.Y,
                -Body1.InverseInertia * R1.X * R1.Y, Body1.InverseInertia * R1.X * R1.X
            );

            Matrix2 k3 = new Matrix2
            (
                Body1.InverseInertia * R2.Y * R2.Y, -Body1.InverseInertia * R2.X * R2.Y,
                -Body1.InverseInertia * R2.X * R2.Y, Body1.InverseInertia * R2.X * R2.X
            );

            // Add together the matrices
            Matrix2 K = Matrix2.Add(k1, Matrix2.Add(k2, k3));
            K = Matrix2.Add(K, Softness);

            // Set it back to the main Matrix
            M = K.Invert();

            // Position correction?
            Vector2 p1 = Vector2.Add(Body1.Position, R1);
            Vector2 p2 = Vector2.Add(Body2.Position, R2);
            Vector2 dp = p2 - p1;

            if (World.PositionCorrection)
            {
                Bias = -BiasFactor * inverseDt * dp;
            }
            else
            {
                Bias = Vector2.Zero;
            }
            
            // Impulse accumulation
            if (World.WarmStarting)
            {
                Body1.Velocity = Vector2.Subtract(Body1.Velocity, Vector2.Multiply(AccumulatedImpulses, Body1.InverseMass));
                Body1.AngularVelocity = Body1.AngularVelocity - Body1.InverseInertia * Cross(R1, AccumulatedImpulses);

                Body2.Velocity = Vector2.Add(Body2.Velocity, Vector2.Multiply(AccumulatedImpulses, Body2.InverseMass));
                Body2.AngularVelocity = Body2.AngularVelocity + Body2.InverseInertia * Cross(R2, AccumulatedImpulses);
            }
            else
            {
                AccumulatedImpulses = Vector2.Zero;
            }
        }

        public void ApplyImpulse()
        {
            Vector2 dv = Body2.Velocity + Cross(Body2.AngularVelocity, R2)
                - Body1.Velocity - Cross(Body1.AngularVelocity, R1);

            Vector2 impulse = Matrix2.Multiply(M, Bias - dv - Softness * AccumulatedImpulses);
            
            Body1.Velocity -= Body1.InverseMass * impulse;
            Body1.AngularVelocity -= Body1.InverseInertia * Cross(R1, impulse);

            Body2.Velocity += Body2.InverseMass * impulse;
            Body2.AngularVelocity += Body2.InverseInertia * Cross(R2, impulse);

            AccumulatedImpulses += impulse;
        }

        private float Cross(Vector2 A, Vector2 B)
        {
            return A.X * B.Y - A.Y * B.Y;
        }

        private Vector2 Cross(float S, Vector2 A)
        {
            return new Vector2(-S * A.Y, -S * A.X);
        }
    }
}
