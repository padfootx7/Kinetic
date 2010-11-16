using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;

namespace Kinetic
{
    public class World
    {
        // Define variables
        private Vector2 gravity;

        private List<Body> bodies;
        private List<Joint> joints;
        
        private Dictionary<ArbiterKey, Arbiter> arbiters;
        private Dictionary<ArbiterKey, Arbiter> arbiterPairs;

        public static bool AccumulateImpulses = true;
        public static bool WarmStarting = true;
        public static bool PositionCorrection = true;

        private int iterations = 10;

        public void Add(Body body)
        {
            bodies.Add(body);
        }

        public void Add(Joint joint)
        {
            joints.Add(joint);
        }

        public void Clear()
        {
            bodies.Clear();
            joints.Clear();
            arbiters.Clear();
        }

        // Broad phase determination of overlaåpping bodies
        public void BroadPhase()
        {
            // Replace by foreach?
            for (int i = 0; i < bodies.Count; i++)
            {
                Body bodyI = bodies[i];

                for (int j = i + 1; j < bodies.Count; ++j)
                {
                    Body bodyJ = bodies[j];

                    // If they both have infinite mass, skip
                    if (bodyI.InverseMass == 0.0f && bodyJ.InverseMass == 0.0f)
                        continue;

                    // Create arbiters
                    Arbiter newArbiter = new Arbiter(bodyI, bodyJ);
                    ArbiterKey newKey = new ArbiterKey(bodyI, bodyJ);

                    // Work out contacts
                    if (newArbiter.NumberOfContacts > 0)
                    {
                        Arbiter arb = arbiters[newKey];
                        if (arb = arbiters.Last)
                        {
                            arbiters.Add(arbiterPairs[newKey], newArbiter);
                        }
                        else
                        {
                            newArbiter.Update(newArbiter.Contacts, newArbiter.NumberOfContacts);
                        }
                    }
                    else
                    {
                        arbiters.Remove(newKey);
                    }
                }
            }
        }

        // Calculate the next step in the physics model
        public void Step(float dt)
        {
            // Get inverse dt (or zero if dt less than 0)
            float inverseDt = dt > 0.0f ? 1.0f / dt : 0.0f;

            // Enable broad phase contact points update and overlapping check
            BroadPhase();

            // Integrate forces
            foreach (Body b in bodies)
            {
                if (b.InverseMass == 0.0f)
                    continue;

                b.Velocity += dt * (gravity.Y + b.InverseMass * b.Force);
                b.AngularVelocity += dt * b.InverseInertia * b.Torque);
            }

            // Perform presteps
            foreach(Arbiter a in arbiters.Values)
            {
                a.Second.Prestep(inverseDt);
            }

            foreach(Joint j in joints)
            {
                j.Prestep(inverseDt);
            }

            // Perform iterations
            for (int i = 0; i < iterations; ++i)
            {
                foreach(Arbiter a in arbiters.Values)
                {
                    a.Second.ApplyImpulses();
                }

                foreach(Joint j in joints)
                {
                    j.ApplyImpulse();
                }
            }

            // Integrate velocities
            foreach(Body b in bodies)
            {
                b.Position += dt * b.Velocity;
                b.Rotation += dt * b.AngularVelocity;

                b.Force = Vector2.Zero;
                b.Torque = 0.0f;
            }
        }

    }
}
