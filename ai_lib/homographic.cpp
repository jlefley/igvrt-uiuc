/*
 *  Homographic.c
 *  
 *
 */


Point_Obj_Prime WorldView:transformPointHomographic(int x, int y)
{
  Point_Obj_Prime coords_prime;

  coords_prime.x_p = (H[0][0]*x) + (H[0][1]*y) + (H[0][2]/* *1 */);
  coords_prime.y_p = (H[1][0]*x) + (H[1][1]*y) + (H[1][2]/* *1 */);
  coords_prime.z_p = (H[2][0]*x) + (H[2][1]*y) + (H[2][2]/* *1 */);

  return coords_prime;
}
