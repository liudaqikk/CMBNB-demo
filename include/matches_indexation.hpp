/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                 CONTRAST MAXIMISATION BRANCH AND BOUND
%
%
% This package contains the source code which implements the
% Contrast maximisation BnB algorithm (CMBnB) in
%
%       Globally Optimal Contrast Maximisation for Event-based  
%                       Motion Estimation
%
% The source code, binaries and demo are supplied for academic use only.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "matches_indexation.h"
#include "util_sort.h"
#include "reg_binary_tree.h"
#include "reg_common.h"
#include "reg_geometry.h"
using namespace Eigen;
using namespace std;


inline
void multiply_aux(reg::Matrix3X &Y, reg::Matrix3 &R,  const reg::Matrix3X &X )
{
    char *chn = (char *)"N";
    double alpha = 1.0, beta = 0.0;
    long int m = 3;
    long int n = X.cols();

    dgemm(chn, chn, &m, &n, &m, &alpha, R.getPr(), &m, X.x, &m, &beta, Y.x, &m);
}

using namespace reg::search;
using namespace reg::geometry;

template <class SSR>
MatchesIndexation<SSR>::MatchesIndexation(
const MatrixXd &X, const MatrixXd &rays, const MatrixXd &MK):
    event(X), ray(rays), intrinic(MK), _size(0)
{
    mxAssert(th>0 && X.cols()==Y.cols(), "invalid input");
}

template <class SSR>
MatchesIndexation<SSR>::MatchesIndexation()
{}

template <class SSR>
int MatchesIndexation<SSR>::evalUpperBound(
        SSR ssr) const
{

  int square_sum = 0;
  for (int i = 1; i <= global_max; i++){
    if (localmax(i) > 0){
      square_sum += localmax(i)*i*i;
    }
  }
  return square_sum;
}

template <class SSR>
int MatchesIndexation<SSR>::evalLowerBound(SSR ssr) const
{
  MatrixXd event_image = ArrayXXd::Zero(rows,cols);
  ArrayXd x = round(reg_points.col(0));
  ArrayXd y = round(reg_points.col(1));

  for (int i = 0; i < event_length; i++){
    if (x(i) > 0 && y(i) > 0 && x(i) <= cols && y(i) <= rows){
      event_image(y(i)-1,x(i)-1) ++ ;
    }
  }
  Map<ArrayXd> vec_image(event_image.data(),event_image.size());
  int contrast = (vec_image*vec_image).sum();
  //cout<<contrast<<endl;
  return contrast;
}

/* compute the upper bound and contrast based on a box */
template <class SSR>
void MatchesIndexation<SSR>::contrast_bound(SSR ssr, int bound_required)
{
  initialization(ssr);
//   auto start = chrono::steady_clock::now();

  warp_event(bound_required);
//   auto end = chrono::steady_clock::now();
//   double t = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
//   cout<<"warp_time: "<<t<<endl;
}

/* Initilization */
template <class SSR>
void MatchesIndexation<SSR>::initialization(SSR ssr)
{
  rows = 180;
  cols = 240;
  _size = event.rows();
  box = MatrixXd::Zero(2,3);
  box<<ssr.ax,ssr.ay,ssr.az,ssr.bx,ssr.by,ssr.bz;
  event_length = _size;
  round_require = FALSE;
  ellipse_require = TRUE;
  VectorXd dif = box.row(1) - box.row(0);
  VectorXd dif2 = dif*dif;
  bound = 0.5*sqrt(dif2.sum());
  c = 0.5*(box.row(1)+box.row(0));
  rows = 180;
  cols = 240;
  reg_points = ArrayXXd::Zero(event_length,3);
  cr = ArrayXXd::Zero(event_length,4);
  global_count = 0;
  localmax = ArrayXd::Zero(7000);
  upper_image = ArrayXXd::Zero(rows,cols);
  upper_contrast = 0;
  global_max = 0;
  radius_limit = 0.05; // box dynamic poster
  //radius_limit = 0.05; // shapes
}

/* compute the warp_event location and the event circle in the event image */
template <class SSR>
void MatchesIndexation<SSR>::warp_event(int bound_required)
{
  double gap_time = event(event_length-1,0) - event(0,0);
  MatrixXd temp_c = MatrixXd::Zero(event_length,3);
  temp_c.col(0) = event.col(0)*c(0)/gap_time;
  temp_c.col(1) = event.col(0)*c(1)/gap_time;
  temp_c.col(2) = event.col(0)*c(2)/gap_time;
  ArrayXd temp_box = bound*event.col(0)/gap_time;
  Axis2R(temp_c);
  rotate();
  if (bound_required == TRUE){
    compute_circle(temp_box);
    ArrayXd lower_x = (cr.col(0)-cr.col(2)).max(1);
    ArrayXd lower_y = (cr.col(1)-cr.col(2)).max(1);
    ArrayXd upper_x = (cr.col(0)+cr.col(2)).min(240);
    ArrayXd upper_y = (cr.col(1)+cr.col(2)).min(180);
    for (int i = 0; i < event_length; i ++){
      pixUcir(i, lower_x(i), lower_y(i), upper_x(i),upper_y(i));
    }
  }
}

/* convert axis angle representation to rotation matrix */
template <class SSR>
void MatchesIndexation<SSR>::Axis2R(MatrixXd temp_c)
{
  VectorXd angle = temp_c.rowwise().norm();

  ArrayXXd axis = temp_c.array().colwise()/angle.array();

  ArrayXd cos_theta = angle.array().cos();
  ArrayXd sin_theta = angle.array().sin();
  ArrayXd wx = axis.col(0);
  ArrayXd wy = axis.col(1);
  ArrayXd wz = axis.col(2);

  // create the rotation matrix in vector form
  r00 = cos_theta + wx * wx * (1 - cos_theta);
  r10 = wz * sin_theta + wx *wy * (1 - cos_theta);
  r20 = -wy * sin_theta + wx * wz * (1 - cos_theta);
  r01 = wx * wy * (1 - cos_theta) - wz * sin_theta;
  r11 = cos_theta + wy * wy * (1 - cos_theta);
  r21 = wx * sin_theta + wy * wz * (1 - cos_theta);
  r02 = wy * sin_theta + wx * wz * (1 - cos_theta);
  r12 = -wx * sin_theta + wy * wz * (1 - cos_theta);
  r22 = cos_theta + wz * wz * (1 - cos_theta);

  // put them back into the rotation matrix;
  int i = 0;
  while (event(i,0) == 0){
    r00(i) = 1;
    r10(i) = 0;
    r20(i) = 0;
    r01(i) = 0;
    r11(i) = 1;
    r21(i) = 0;
    r02(i) = 0;
    r12(i) = 0;
    r22(i) = 1;
    i++;
  }


  /* for debugging */
  //cout<<angle.size()<<endl;
  //cout<<Rot.row(0)<<endl;
}

/* warp event to new coordinate given rotation matrix */
template <class SSR>
void MatchesIndexation<SSR>::rotate()
{
  // compute the rotated rays
  rotate_rx = r00 * ray.col(0).array() + r01 * ray.col(1).array() + r02 * ray.col(2).array();
  rotate_ry = r10 * ray.col(0).array() + r11 * ray.col(1).array() + r12 * ray.col(2).array();
  rotate_rz = r20 * ray.col(0).array() + r21 * ray.col(1).array() + r22 * ray.col(2).array();

  //project to image plane
  ArrayXd rotate_x = rotate_rx*intrinic(0,0) + rotate_ry*intrinic(0,1) + rotate_rz*intrinic(0,2);
  ArrayXd rotate_y = rotate_rx*intrinic(1,0) + rotate_ry*intrinic(1,1) + rotate_rz*intrinic(1,2);
  ArrayXd rotate_z = rotate_rx*intrinic(2,0) + rotate_ry*intrinic(2,1) + rotate_rz*intrinic(2,2);
  reg_points.col(0) = rotate_x/rotate_z;
  reg_points.col(1) = rotate_y/rotate_z;
  reg_points.col(2) = event.col(3).array();
  /* for debugging */
  //cout<<r00(1)<<endl;
  //cout<<rotate_rx(1)<<' '<< rotate_ry(1)<<' '<<rotate_rz(1)<<endl;
  //cout<<rotate_rx(event_length-1)<<' '<< rotate_ry(event_length-1)<<' '<<rotate_rz(event_length-1)<<endl;
  //cout<<intrinic<<endl;
  //cout<<rotate_x(1)<<' '<<rotate_y(1)<<' '<<rotate_z(1)<<endl;
  //cout<<reg_points(0,0)<<' '<< reg_points(0,1)<<' '<<reg_points(0,2)<<endl;
  //cout<<reg_points(event_length-1,0)<<' '<< reg_points(event_length-1,1)<<' '<<reg_points(event_length-1,2)<<endl;
}

/* compute the center and radius of the event circles */
template <class SSR>
void MatchesIndexation<SSR>::compute_circle(ArrayXd temp_box)
{
  // convert variable for further calculation
  Vector3d n(0,0,1);
  ArrayXd r = temp_box.sin();
  ArrayXXd rotate_ray(event_length,3);
  rotate_ray << rotate_rx, rotate_ry, rotate_rz;

  //cross product
  MatrixXd cro = ray.matrix().rowwise().cross(n);
  ArrayXXd EF(event_length,3);
  EF.col(0) = rotate_ray.col(1)*cro.array().col(2) - rotate_ray.col(2)*cro.array().col(1);
  EF.col(1) = rotate_ray.col(0)*cro.array().col(2) - rotate_ray.col(2)*cro.array().col(0);
  EF.col(2) = rotate_ray.col(0)*cro.array().col(1) - rotate_ray.col(1)*cro.array().col(0);

  // compute the shortest EF, which is the cross product of EF and n
  EF.col(0) = rotate_ray.col(1)*cro.array().col(2) - rotate_ray.col(2)*cro.array().col(1);
  EF.col(1) = rotate_ray.col(0)*cro.array().col(2) - rotate_ray.col(2)*cro.array().col(0);
  EF.col(2) = rotate_ray.col(0)*cro.array().col(1) - rotate_ray.col(1)*cro.array().col(0);


  // compute the longest ray in 3D plane
  ArrayXd nEF = EF.matrix().rowwise().norm().array();
  ArrayXXd FK(event_length,3);
  ArrayXXd EK(event_length,3);
  EK.col(0) = rotate_ray.col(0) + r*EF.col(0)/nEF;
  EK.col(1) = rotate_ray.col(1) - r*EF.col(1)/nEF;
  EK.col(2) = rotate_ray.col(2) + r*EF.col(2)/nEF;
  FK.col(0) = rotate_ray.col(0) - r*EF.col(0)/nEF;
  FK.col(1) = rotate_ray.col(1) + r*EF.col(1)/nEF;
  FK.col(2) = rotate_ray.col(2) - r*EF.col(2)/nEF;

  // compute the shortest ray in 3D plane
  ArrayXXd CEF = cro.array();
  ArrayXd nCEF = CEF.matrix().rowwise().norm().array();
  ArrayXXd CFK(event_length,3);
  ArrayXXd CEK(event_length,3);
  CEK.col(0) = rotate_ray.col(0) + r*CEF.col(0)/nCEF;
  CEK.col(1) = rotate_ray.col(1) - r*CEF.col(1)/nCEF;
  CEK.col(2) = rotate_ray.col(2) + r*CEF.col(2)/nCEF;
  CFK.col(0) = rotate_ray.col(0) - r*CEF.col(0)/nCEF;
  CFK.col(1) = rotate_ray.col(1) + r*CEF.col(1)/nCEF;
  CFK.col(2) = rotate_ray.col(2) - r*CEF.col(2)/nCEF;


  //project the ray back to project plane;
  ArrayXXd EE(event_length,3);
  ArrayXXd FF(event_length,3);
  EE.col(0) = EK.col(0)*intrinic(0,0) + EK.col(1)*intrinic(0,1) + EK.col(2)*intrinic(0,2);
  EE.col(1) = EK.col(0)*intrinic(1,0) + EK.col(1)*intrinic(1,1) + EK.col(2)*intrinic(1,2);
  EE.col(2) = EK.col(0)*intrinic(2,0) + EK.col(1)*intrinic(2,1) + EK.col(2)*intrinic(2,2);
  FF.col(0) = FK.col(0)*intrinic(0,0) + FK.col(1)*intrinic(0,1) + FK.col(2)*intrinic(0,2);
  FF.col(1) = FK.col(0)*intrinic(1,0) + FK.col(1)*intrinic(1,1) + FK.col(2)*intrinic(1,2);
  FF.col(2) = FK.col(0)*intrinic(2,0) + FK.col(1)*intrinic(2,1) + FK.col(2)*intrinic(2,2);

  //proejct the shorest back to project plane;
  ArrayXXd CEE(event_length,3);
  ArrayXXd CFF(event_length,3);
  CEE.col(0) = CEK.col(0)*intrinic(0,0) + CEK.col(1)*intrinic(0,1) + CEK.col(2)*intrinic(0,2);
  CEE.col(1) = CEK.col(0)*intrinic(1,0) + CEK.col(1)*intrinic(1,1) + CEK.col(2)*intrinic(1,2);
  CEE.col(2) = CEK.col(0)*intrinic(2,0) + CEK.col(1)*intrinic(2,1) + CEK.col(2)*intrinic(2,2);
  CFF.col(0) = CFK.col(0)*intrinic(0,0) + CFK.col(1)*intrinic(0,1) + CFK.col(2)*intrinic(0,2);
  CFF.col(1) = CFK.col(0)*intrinic(1,0) + CFK.col(1)*intrinic(1,1) + CFK.col(2)*intrinic(1,2);
  CFF.col(2) = CFK.col(0)*intrinic(2,0) + CFK.col(1)*intrinic(2,1) + CFK.col(2)*intrinic(2,2);

  // compute everything in the projected plane
  ArrayXXd E(event_length,2);
  ArrayXXd F(event_length,2);
  E.col(0) = EE.col(0)/EE.col(2);
  E.col(1) = EE.col(1)/EE.col(2);
  F.col(0) = FF.col(0)/FF.col(2);
  F.col(1) = FF.col(1)/FF.col(2);

  // compute the shortest pojrct in the projected plane
  ArrayXXd CE(event_length,2);
  ArrayXXd CF(event_length,2);
  CE.col(0) = CEE.col(0)/CEE.col(2);
  CE.col(1) = CEE.col(1)/CEE.col(2);
  CF.col(0) = CFF.col(0)/CFF.col(2);
  CF.col(1) = CFF.col(1)/CFF.col(2);

  cr.col(2) = (E-F).matrix().rowwise().norm()/2;
  cr.col(0) = 0.5*(E.col(0)+F.col(0));
  cr.col(1) = 0.5*(E.col(1)+F.col(1));
  cr.col(3) = (CE-CF).matrix().rowwise().norm()/2;

  /* for debugging */
  //cout<<cro.row(event_length-3)<<endl;
  //cout<<EF.row(event_length-3)<<endl;
  //cout<<rotate_ray.row(event_length-1)<<endl;
  //cout<<ray.row(event_length-1)<<endl;
  //cout<<r00(event_length-1)<<' '<<r01(event_length-1)<<' '<<r02(event_length-1)<<endl;
  //cout<<r10(event_length-1)<<' '<<r11(event_length-1)<<' '<<r12(event_length-1)<<endl;
  //cout<<r20(event_length-1)<<' '<<r21(event_length-1)<<' '<<r22(event_length-1)<<endl;
  //cout<<ray.row(1)<<endl;

  //cout<<EF.row(event_length-1)<<endl;
  //cout<<CEF.row(event_length-1)<<endl;
}

/* compute upper bound of event image */
template <class SSR>
void MatchesIndexation<SSR>::pixUcir(int i, double lx1, double ly1, double ux1, double uy1){
  int ux = min((int) round(ux1),240);
  int lx = max((int) round(lx1),1);
  int uy = min((int) round(uy1),180);
  int ly = max((int) round(ly1),1);
  int x = ux - lx + 1;
  int y = uy - ly + 1;
  int lm = 0;
  ArrayXXi im;
  if (ux > 0 && uy > 0 && lx <= 240 && ly <= 180){
    if (x > 1 ||  y > 1){
      im = midptellipse(cr(i,2),cr(i,3),cr(i,1)-ly,cr(i,0)-lx,y,x);
      lm = update_image(y, x, lx, ly, im);
        //cout<<"i: "<<i<<endl;
        //cout<<"cr: "<<cr.row(i)<<endl;
        //cout<<"im: "<<im<<endl;
        //cout<<"xylxlyuxuy: "<<x<<' '<<y<<' '<<lx<<' '<<ly<<' '<< ux<<' '<<uy<<endl;
    }else{
      upper_image(ly-1,lx-1) ++ ;
      lm = upper_image(ly-1,lx-1);
      if (lm>global_max){
        global_max = lm;
      }
    }
    compute_localmax(lm);
  }
}
template <class SSR>
ArrayXXi MatchesIndexation<SSR>::midptellipse(double rx, double ry, double xc, double yc,int row, int col){
    ArrayXXi im = ArrayXXi::Zero(row,col);
    double dx, dy, d1, d2, y;
    double xx = 0;
    if (ry < radius_limit){
      ry = 0;
    }
    if (rx < radius_limit){
      rx = 0;
    }
    //cout << xx << endl;
    if (round_require == TRUE){
      ry = round(ry);
      rx = round(rx);
      xc = round(xc);
      yc = round(yc);
    }
    y = ry;

    // Initial decision parameter of region 1
    d1 = (ry * ry) - (rx * rx * ry) +
                     (0.25 * rx * rx);
    dx = 2 * ry * ry * xx;
    dy = 2 * rx * rx * y;
    // For region 1
    while (dx < dy)
    {

        // Print points based on 4-way symmetry
        im(max(min((int)round(xx+xc),row-1),0), max(min((int)round(y+yc),col-1),0)) = 1;
        im(max(min((int)round(xc-xx),row-1),0), max(min((int)round(yc+y),col-1),0)) = 1;
        im(max(min((int)round(xx+xc),row-1),0), max(min((int)round(yc-y),col-1),0)) = 1;
        im(max(min((int)round(xc-xx),row-1),0), max(min((int)round(yc-y),col-1),0)) = 1;

        // Checking and updating value of
        // decision parameter based on algorithm

        if (d1 < 0)
        {
            xx = xx + 1;
            dx = dx + (2 * ry * ry);
            d1 = d1 + dx + (ry * ry);
        }
        else{
            xx = xx + 1;
            y = y - 1;
            dx = dx + (2 * ry * ry);
            dy = dy - (2 * rx * rx);
            d1 = d1 + dx - dy + (ry * ry);
        }
    }

    // Decision parameter of region 2
    d2 = ((ry * ry) * ((xx + 0.5) * (xx + 0.5))) +
         ((rx * rx) * ((y - 1) * (y - 1))) -
          (rx * rx * ry * ry);

    // Plotting points of region 2
    while (y >= 0)
    {

        // Print points based on 4-way symmetry
        im(max(min((int)round(xx+xc),row-1),0), max(min((int)round(y+yc),col-1),0)) = 1;
        im(max(min((int)round(xc-xx),row-1),0), max(min((int)round(yc+y),col-1),0)) = 1;
        im(max(min((int)round(xx+xc),row-1),0), max(min((int)round(yc-y),col-1),0)) = 1;
        im(max(min((int)round(xc-xx),row-1),0), max(min((int)round(yc-y),col-1),0)) = 1;

        // Checking and updating parameter
        // value based on algorithm
        if (d2 > 0)
        {
            y = y - 1;
            dy = dy - (2 * rx * rx);
            d2 = d2 + (rx * rx) - dy;
        }
        else
        {
            y = y - 1;
            xx = xx + 1;
            dx = dx + (2 * ry * ry);
            dy = dy - (2 * rx * rx);
            d2 = d2 + dx - dy + (rx * rx);
        }
    }
    return im;
}

template <class SSR>
void MatchesIndexation<SSR>::compute_localmax(int lm){
  while(lm > 0){
    if (lm == 1){
      localmax(lm) = localmax(lm) + 1;
    }
    else if (localmax(lm-1) > 0 ){
      localmax(lm-1) = localmax(lm-1) - 1;
      localmax(lm) = localmax(lm) + 1;
      break;
    }
    lm --;
  }
}

/* project the events into the event image */
template <class SSR>
int MatchesIndexation<SSR>::update_image(int y, int x, int lx, int ly, ArrayXXi im){
  int lm = 0;
  int start;
  int count;
  int total;
  // compute the pixel intersection
  for (int j = 0; j < y; j ++){
    total = im.row(j).sum();
    start = FALSE;
    count = 0;
    if (total == 0){
      continue;
    }
    for (int k = 0; k < x; k ++){
      if (im(j,k) == TRUE){
        start = TRUE;
        count ++;
      }
      if (start == TRUE &&
      j + ly > 0 &&
      j + ly <= 180 &&
      k + lx > 0 &&
      k + lx <= 240){
        //if (i == 1){
        //  cout<<"xy_uxuy: "<<round(cr(i,0))<<' '<<round(cr(i,1))<<' '<<j+ly<<' '<<k+lx<<endl;
        //}
        //cout<<j+ly-1<<' '<<k+lx-1<<endl;
        //cout<<upper_image.rows()<<' '<<upper_image.cols()<<endl;
        upper_image(j+ly-1,k+lx-1) = upper_image(j+ly-1,k+lx-1) + 1;
        if (upper_image(j+ly-1,k+lx-1) > lm){
          lm = upper_image(j+ly-1,k+lx-1);
          if (lm>global_max){
            global_max = lm;
          }
        }
        //global_count++;
        //cout<<global_count<<endl;
      }
      if (count == total){
        break;
      }
    }
  }
  return lm;
}
