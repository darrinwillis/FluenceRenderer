#include "matrix2x2.h"

#include <iostream>
#include <cmath>

using namespace std;

namespace CMU462 {

  double& Matrix2x2::operator()( int i, int j ) {
    return entries[j][i];
  }

  const double& Matrix2x2::operator()( int i, int j ) const {
    return entries[j][i];
  }

  Vector2D& Matrix2x2::operator[]( int j ) {
      return entries[j];
  }

  const Vector2D& Matrix2x2::operator[]( int j ) const {
    return entries[j];
  }

  void Matrix2x2::zero( double val ) {
    // sets all elements to val
    entries[0] = entries[1] = Vector2D( val, val );
  }

  double Matrix2x2::det( void ) const {
    const Matrix2x2& A( *this );

    return A(0,0)*A(1,1) - A(1,0)*A(0,1);
  }

  double Matrix2x2::norm( void ) const {
    return sqrt( entries[0].norm2() +
                 entries[1].norm2() );
  }

  Matrix2x2 Matrix2x2::operator-( void ) const {

   // returns -A
    const Matrix2x2& A( *this );
    Matrix2x2 B;

    B(0,0) = -A(0,0); B(0,1) = -A(0,1);
    B(1,0) = -A(1,0); B(1,1) = -A(1,1);

    return B;
  }

  void Matrix2x2::operator+=( const Matrix2x2& B ) {

    Matrix2x2& A( *this );
    double* Aij = (double*) &A;
    const double* Bij = (const double*) &B;

    *Aij++ += *Bij++;
    *Aij++ += *Bij++;
    *Aij++ += *Bij++;
    *Aij++ += *Bij++;
  }

  Matrix2x2 Matrix2x2::operator-( const Matrix2x2& B ) const {
    const Matrix2x2& A( *this );
    Matrix2x2 C;

    for( int i = 0; i < 2; i++ )
    for( int j = 0; j < 2; j++ )
    {
       C(i,j) = A(i,j) - B(i,j);
    }

    return C;
  }

  Matrix2x2 Matrix2x2::operator*( double c ) const {
    const Matrix2x2& A( *this );
    Matrix2x2 B;

    for( int i = 0; i < 2; i++ )
    for( int j = 0; j < 2; j++ )
    {
       B(i,j) = c*A(i,j);
    }

    return B;
  }

  Matrix2x2 operator*( double c, const Matrix2x2& A ) {

    Matrix2x2 cA;
    const double* Aij = (const double*) &A;
    double* cAij = (double*) &cA;

    *cAij++ = c * (*Aij++);
    *cAij++ = c * (*Aij++);
    *cAij++ = c * (*Aij++);
    *cAij++ = c * (*Aij++);

    return cA;
  }

  Matrix2x2 Matrix2x2::operator*( const Matrix2x2& B ) const {
    const Matrix2x2& A( *this );
    Matrix2x2 C;

    for( int i = 0; i < 2; i++ )
    for( int j = 0; j < 2; j++ )
    {
       C(i,j) = 0.;

       for( int k = 0; k < 2; k++ )
       {
          C(i,j) += A(i,k)*B(k,j);
       }
    }

    return C;
  }

  Vector2D Matrix2x2::operator*( const Vector2D& x ) const {
    return x[0]*entries[0] +
           x[1]*entries[1] ;
  }

  Matrix2x2 Matrix2x2::T( void ) const {
    const Matrix2x2& A( *this );
    Matrix2x2 B;

    for( int i = 0; i < 2; i++ )
    for( int j = 0; j < 2; j++ )
    {
       B(i,j) = A(j,i);
    }

    return B;
  }

  Matrix2x2 Matrix2x2::inv( void ) const {
    const Matrix2x2& A( *this );
    Matrix2x2 B;

    B(0,0) = -A(1,2)*A(2,1) + A(1,1)*A(2,2); B(0,1) =  A(0,2)*A(2,1) - A(0,1)*A(2,2); B(0,2) = -A(0,2)*A(1,1) + A(0,1)*A(1,2);
    B(1,0) =  A(1,2)*A(2,0) - A(1,0)*A(2,2); B(1,1) = -A(0,2)*A(2,0) + A(0,0)*A(2,2); B(1,2) =  A(0,2)*A(1,0) - A(0,0)*A(1,2);
    B(2,0) = -A(1,1)*A(2,0) + A(1,0)*A(2,1); B(2,1) =  A(0,1)*A(2,0) - A(0,0)*A(2,1); B(2,2) = -A(0,1)*A(1,0) + A(0,0)*A(1,1);

    B(0,0) =  B(1,1);
    B(1,0) = -B(1,0);
    B(0,1) = -B(0,1);
    B(1,1) =  B(0,0);

    B /= det();

    return B;
  }

  void Matrix2x2::operator/=( double x ) {
    Matrix2x2& A( *this );
    double rx = 1./x;

    for( int i = 0; i < 2; i++ )
    for( int j = 0; j < 2; j++ )
    {
       A( i, j ) *= rx;
    }
  }

  Matrix2x2 Matrix2x2::identity( void ) {
    Matrix2x2 B;

    B(0,0) = 1.; B(0,1) = 0.;
    B(1,0) = 0.; B(1,1) = 1.;

    return B;
  }

  Matrix2x2 Matrix2x2::rotation( double theta ) {
    Matrix2x2 B;

    B(0,0) =  cos(theta); B(0,1) = sin(theta);
    B(1,0) = -sin(theta); B(1,1) = cos(theta);

    return B;
  }

  double Matrix2x2::getRotation( void ) const
  {
     double aCosTheta = entries[0][0];
     double aSinTheta = entries[1][0];

     return atan2( aSinTheta, aCosTheta );
  }

  Matrix2x2 Matrix2x2::crossProduct( const Vector2D& u ) {
    Matrix2x2 B;

    B(0,0) =   0.;  B(0,1) =  u.x;
    B(1,0) = -u.y;  B(1,1) =   0.;

    return B;
  }

  Matrix2x2 outer( const Vector2D& u, const Vector2D& v ) {
    Matrix2x2 B;
    double* Bij = (double*) &B;

    *Bij++ = u.x*v.x;
    *Bij++ = u.y*v.x;
    *Bij++ = u.x*v.y;
    *Bij++ = u.y*v.y;

    return B;
  }

  std::ostream& operator<<( std::ostream& os, const Matrix2x2& A ) {
    for( int i = 0; i < 2; i++ )
    {
       os << "[ ";

       for( int j = 0; j < 2; j++ )
       {
          os << A(i,j) << " ";
       }

       os << "]" << std::endl;
    }

    return os;
  }

  Vector2D& Matrix2x2::column( int i ) {
    return entries[i];
  }

  const Vector2D& Matrix2x2::column( int i ) const {
    return entries[i];
  }
}
