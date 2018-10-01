/*
skeleton.h

Definition of the skeleton. 

Written by Jehee Lee

Revision 1 - Steve Lin (CMU), Jan 14, 2002
Revision 2 - Alla Safonova and Kiran Bhat (CMU), Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao (USC), Feb, 2012

*/

#ifndef _VECTORS_H
#define _VECTORS_H


class vectors
{
  // negation
  friend vectors    operator-( vectors const& );


  // addtion
  friend vectors    operator+( vectors const&, vectors const& );

  // subtraction
  friend vectors    operator-( vectors const&, vectors const& );

  // dot product
  friend double    operator%( vectors const&, vectors const& );

  // cross product
  friend vectors    operator*( vectors const&, vectors const& );

  // scalar Multiplication
  friend vectors    operator*( vectors const&, double );

  // scalar Division
  friend vectors    operator/( vectors const&, double );


  friend double    len( vectors const& );
  friend vectors	normalize( vectors const& );

  friend double       angle( vectors const&, vectors const& );

  // member functions
public:
  // constructors
  vectors() {}
  vectors( double x, double y, double z ) { p[0]=x; p[1]=y; p[2]=z; }
  vectors( double a[3] ) { p[0]=a[0]; p[1]=a[1]; p[2]=a[2]; }
  ~vectors() {};

  // inquiry functions
  double& operator[](int i) { return p[i];}

  double x() const { return p[0]; };
  double y() const { return p[1]; };
  double z() const { return p[2]; };
  void   getValue( double d[3] ) { d[0]=p[0]; d[1]=p[1]; d[2]=p[2]; }
  void   setValue( double d[3] ) { p[0]=d[0]; p[1]=d[1]; p[2]=d[2]; }

  double getValue( int n ) const { return p[n]; }
  vectors setValue( double x, double y, double z )
  { p[0]=x, p[1]=y, p[2]=z; return *this; }
  double setValue( int n, double x )
  { return p[n]=x; }

  double length() const;

  // change functions
  void set_x( double x ) { p[0]=x; };
  void set_y( double x ) { p[1]=x; };
  void set_z( double x ) { p[2]=x; };

  //data members
  double p[3]; //X, Y, Z components of the vector
};

#endif

