/*

Revision 1 - Steve Lin (CMU), Jan 14, 2002
Revision 2 - Alla Safonova and Kiran Bhat (CMU), Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao (USC), Feb, 2012

*/
#ifndef _POSTURE_H
#define _POSTURE_H

#include "vectors.h"
#include "types.h"

//Root position and all bone rotation angles (including root) 
struct Posture
{
public:
  //Root position (x, y, z)		
  vectors root_pos;								

  //Euler angles (thetax, thetay, thetaz) of all bones in their local coordinate system.
  //If a particular bone does not have a certain degree of freedom, 
  //the corresponding rotation is set to 0.
  //The order of the bones in the array corresponds to their ids in .ASf file: root, lhipjoint, lfemur, ...
  vectors bone_rotation[MAX_BONES_IN_ASF_FILE];
  
  // bones that are translated relative to parents (resulting in gaps) (rarely used)
  vectors bone_translation[MAX_BONES_IN_ASF_FILE];

  // bones that change length during the motion (rarely used)
  vectors bone_length[MAX_BONES_IN_ASF_FILE];
};

#endif

