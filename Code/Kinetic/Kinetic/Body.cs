using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Kinetic
{
    public class Body
    {
        // The many physics properties of a body
        public Vector2 Position;
        public Vector2 Velocity;
        public Vector2 Force;
        public Vector2 Size;

        public float Rotation;
        public float AngularVelocity;
        public float Torque;
        public float Mass;
        public float Inertia;
        public float Friction = 0.2f;

        public float InverseInertia;
        public float InverseMass;

        public Body(Vector2 size, float mass = float.MaxValue)
        {
            // Set size of the object
            Size = size;

            // Calculate inversemass and inertia if object != infinite mass
            if (mass < float.MaxValue)
            {
                InverseMass = 1.0f / mass;
                Inertia = mass * (Size.X + Size.Y * Size.Y) / 12.0f;
                InverseInertia = 1.0f / Inertia;
            }
            else
            {
                InverseMass = 0.0f;
                Inertia = float.MaxValue;
                InverseInertia = 0.0f;
            }
        }

        public void AddForce(Vector2 f)
        {
            Force = Vector2.Add(Force, f);
        }

    }
}
