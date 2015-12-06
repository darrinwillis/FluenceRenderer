// Given a time between 0 and 1, evaluates a cubic polynomial with
// the given endpoint and tangent values at the beginning (0) and
// end (1) of the interval.  Optionally, one can request a derivative
// of the spline (0=no derivative, 1=first derivative, 2=2nd derivative).
template <class T>
inline T Spline<T>::cubicSplineUnitInterval(
      const T& position0,
      const T& position1,
      const T& tangent0,
      const T& tangent1,
      double normalizedTime,
      int derivative )
{
    double t, t2, t3;
    double h00, h10, h01, h11;
    t = normalizedTime;
    t2 = t * t;
    t3 = t2 * t;

    switch (derivative) {
        case 0:
            h00 =  2.0*t3 - 3.0*t2 +     1;
            h10 =      t3 - 2.0*t2 + t;
            h01 = -2.0*t3 + 3.0*t2;
            h11 =      t3 -     t2;
            break;
        case 1:
            h00 =  6.0*t2 - 6.0*t;
            h10 =  3.0*t2 - 4.0*t + 1;
            h01 = -6.0*t2 + 6.0*t;
            h11 =  3.0*t2 - 2.0*t;
            break;
        case 2:
            h00 =  12.0*t - 6.0;
            h10 =   6.0*t - 4.0;
            h01 = -12.0*t + 6.0;
            h11 =   6.0*t - 2.0;
            break;
        default:
            assert(0);
    }

    T p = h00 * position0 + h10 * tangent0 +
          h01 * position1 + h11 * tangent1;
    return p;
}
            
// Returns a state interpolated between the values directly before and after the given time.
template <class T>
inline T Spline<T>::evaluate( double time, int derivative )
{
    // Check for no knots
    if (knots.empty()) {
        return T();
    }
    if (knots.size() == 1) {
        if (derivative == 0) {
            return knots.begin()->second;
        } else {
            return T();
        }
    }
    KnotCIter firstIter = knots.begin();
    KnotCIter lastIter = --knots.end();

    // check if the time is before the first knot
    if (time <= firstIter->first) {
        if (derivative == 0) {
            return firstIter->second;
        } else {
            return T();
        }
    }
    // check if the time is after the last knot
    if (time >= lastIter->first) {
        if (derivative == 0) {
            return lastIter->second;
        } else {
            return T();
        }
    }

    // get the 4 knots we need
    // note: ki1 is 1 to the left and ki2 is 1 to the right, etc.
    KnotCIter ki1, ki2;
    T p0, p1, p2, p3;
    double t0, t1, t2,t3;
    ki2 = knots.upper_bound(time);
    ki1 = ki2;
    ki1--;
    p1 = ki1->second;
    p2 = ki2->second;
    t1 = ki1->first;
    t2 = ki2->first;
    // If ki1 is the first iter, we need to mirror the difference to ki2
    if (ki1 == firstIter) {
        p0 = p1 - (p2 - p1);
        t0 = t1 - (t2 - t1);
    } else {
        KnotCIter ki0 = ki1;
        ki0--;
        p0 = ki0->second;
        t0 = ki0->first;
    }
    
    if (ki2 == lastIter) {
        p3 = p2 + (p2 - p1);
        t3 = t2 + (t2 - t1);
    } else {
        KnotCIter ki3 = ki2;
        ki3++;
        p3 = ki3->second;
        t3 = ki3->first;
    }
    // We now have p0..p3, and t0..t3

    // Get the positions and tangents
    T m1, m2;

    m1 = (p2 - p0) / (t2 - t0);
    m2 = (p3 - p1) / (t3 - t1);

    double normT = (time - t1) / (t2 - t1);
    T result = cubicSplineUnitInterval(p1, p2, m1 * (t3 - t2), m2 * (t3 - t2), normT, derivative);
    switch (derivative) {
        case 0:
            break;
        case 1:
            result /= (t2 - t1);
            break;
        case 2:
            result /= ((t2 - t1) * (t2 - t1));
            break;
        default:
            assert(0);
    }
    return result;
}

// Removes the knot closest to the given time,
//    within the given tolerance..
// returns true iff a knot was removed.
template <class T>
inline bool Spline<T>::removeKnot(double time, double tolerance )
{
   // Empty maps have no knots.
   if( knots.size() < 1 )
   {
      return false;
   }

   // Look up the first element > or = to time.
   typename std::map<double, T>::iterator t2_iter = knots.lower_bound(time);
   typename std::map<double, T>::iterator t1_iter;
   t1_iter = t2_iter;
   t1_iter--;

   if( t2_iter == knots.end() )
   {
      t2_iter = t1_iter;
   }

   // Handle tolerance bounds,
   // because we are working with floating point numbers.
   double t1 = (*t1_iter).first;
   double t2 = (*t2_iter).first;

   double d1 = fabs(t1 - time);
   double d2 = fabs(t2 - time);


   if(d1 < tolerance && d1 < d2)
   {
      knots.erase(t1_iter);
      return true;
   }

   if(d2 < tolerance && d2 < d1)
   {
      knots.erase(t2_iter);
      return t2;
   }

   return false;
}

// Sets the value of the spline at a given time (i.e., knot),
// creating a new knot at this time if necessary.
template <class T>
inline void Spline<T>::setValue( double time, T value )
{
   knots[ time ] = value;
}

template <class T>
inline T Spline<T>::operator()( double time )
{
   return evaluate( time );
}
