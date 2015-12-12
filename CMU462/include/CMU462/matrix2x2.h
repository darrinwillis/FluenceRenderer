#ifndef CMU462_MATRIX2X2_H
#define CMU462_MATRIX2X2_H

#include "CMU462.h"
#include "vector2D.h"

#include <iosfwd>

namespace CMU462 {

/**
 * Defines a 2x2 matrix.
 * 2x2 matrices are extremely useful in computer graphics.
 */
class Matrix2x2 {

  public:

  // The default constructor.
  Matrix2x2(void) { }

  // Constructor for row major form data.
  // Transposes to the internal column major form.
  // REQUIRES: data should be of size 4 for a 2 by 2 matrix..
  Matrix2x2(double * data)
  {
    for( int i = 0; i < 2; i++ ) {
      for( int j = 0; j < 2; j++ ) {
	        // Transpostion happens within the () query.
	        (*this)(i,j) = data[i*2 + j];
      }
    }
  }

  /**
   * Sets all elements to val.
   */
  void zero(double val = 0.0 );

  /**
   * Returns the determinant of A.
   */
  double det( void ) const;

  /**
   * Returns the Frobenius norm of A.
   */
  double norm( void ) const;

  /**
   * Returns the 2x2 identity matrix.
   */
  static Matrix2x2 identity( void );

  /**
   * Returns matrix encoding a 2D counter-clockwise rotation by the angle theta in homogeneous coordinates. The angle is given in Radians.
   */
  static Matrix2x2 rotation( double theta );

  /**
   * Returns a matrix representing the (left) cross product with u.
   */
  static Matrix2x2 crossProduct( const Vector2D& u );

  /**
   * Assuming this matrix represents a 2D homogeneous transformation,
   * returns the rotation it encodes as an angle in radians.
   */
  double getRotation( void ) const;

  /**
   * Returns the ith column.
   */
        Vector2D& column( int i );
  const Vector2D& column( int i ) const;

  /**
   * Returns the transpose of A.
   */
  Matrix2x2 T( void ) const;

  /**
   * Returns the inverse of A.
   */
  Matrix2x2 inv( void ) const;

  // accesses element (i,j) of A using 0-based indexing
        double& operator()( int i, int j );
  const double& operator()( int i, int j ) const;

  // accesses the ith column of A
        Vector2D& operator[]( int i );
  const Vector2D& operator[]( int i ) const;

  // increments by B
  void operator+=( const Matrix2x2& B );

  // returns -A
  Matrix2x2 operator-( void ) const;

  // returns A-B
  Matrix2x2 operator-( const Matrix2x2& B ) const;

  // returns c*A
  Matrix2x2 operator*( double c ) const;

  // returns A*B
  Matrix2x2 operator*( const Matrix2x2& B ) const;

  // returns A*x
  Vector2D operator*( const Vector2D& x ) const;

  // divides each element by x
  void operator/=( double x );

  protected:

  // column vectors
  Vector2D entries[2];

}; // class Matrix2x2

// returns the outer product of u and v
Matrix2x2 outer( const Vector2D& u, const Vector2D& v );

// returns c*A
Matrix2x2 operator*( double c, const Matrix2x2& A );

// prints entries
std::ostream& operator<<( std::ostream& os, const Matrix2x2& A );

} // namespace CMU462

#endif // CMU462_MATRIX2X2_H
