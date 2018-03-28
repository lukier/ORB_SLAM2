#ifndef G2O_EXTENSIONS_H
#define G2O_EXTENSIONS_H

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/base_unary_edge.h>

namespace g2o
{

namespace internal
{
    static inline Vector2 project2d(const Vector3& v)  {
        Vector2 res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
    
    static inline Vector3 unproject2d(const Vector2& v)  {
        Vector3 res;
        res(0) = v(0);
        res(1) = v(1);
        res(2) = 1;
        return res;
    }
}
 
#if 0 // now in g20
class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2, VertexSBAPointXYZ, VertexSE3Expmap>()
    {
        
    }
    
    bool read(std::istream& is)
    {
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
            return true;
    }
    
    bool write(std::ostream& os) const
    {
        
        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }
        
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
            return os.good();
    }
    
    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Vector2 obs(_measurement);
        _error = obs-cam_project(v1->estimate().map(v2->estimate()));
    }
    
    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate()))(2)>0.0;
    }
    
    
    virtual void linearizeOplus()
    {
        VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Vector3 xyz = vi->estimate();
        Vector3 xyz_trans = T.map(xyz);
        
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;
        
        Eigen::Matrix<double,2,3> tmp;
        tmp(0,0) = fx;
        tmp(0,1) = 0;
        tmp(0,2) = -x/z*fx;
        
        tmp(1,0) = 0;
        tmp(1,1) = fy;
        tmp(1,2) = -y/z*fy;
        
        _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();
        
        _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
        _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
        _jacobianOplusXj(0,2) = y/z *fx;
        _jacobianOplusXj(0,3) = -1./z *fx;
        _jacobianOplusXj(0,4) = 0;
        _jacobianOplusXj(0,5) = x/z_2 *fx;
        
        _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
        _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
        _jacobianOplusXj(1,2) = -x/z *fy;
        _jacobianOplusXj(1,3) = 0;
        _jacobianOplusXj(1,4) = -1./z *fy;
        _jacobianOplusXj(1,5) = y/z_2 *fy;
    }
    
    Vector2 cam_project(const Vector3 & trans_xyz) const
    {
        Vector2 proj = internal::project2d(trans_xyz);
        Vector2 res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }
    
    double fx, fy, cx, cy;
};
#endif

#if 0 // now in g20
class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeStereoSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3, VertexSBAPointXYZ, VertexSE3Expmap>()
    {
        
    }
    
    bool read(std::istream& is)
    {
        for (int i=0; i<=3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
            return true;
    }
    
    bool write(std::ostream& os) const
    {
        
        for (int i=0; i<=3; i++){
            os << measurement()[i] << " ";
        }
        
        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++){
                os << " " <<  information()(i,j);
            }
            return os.good();
    }
    
    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Vector3 obs(_measurement);
        _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
    }
    
    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate()))(2)>0.0;
    }
    
    
    virtual void linearizeOplus()
    {
        VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Vector3 xyz = vi->estimate();
        Vector3 xyz_trans = T.map(xyz);
        
        const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();
        
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;
        
        _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
        _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
        _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;
        
        _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
        _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
        _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;
        
        _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
        _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
        _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;
        
        _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
        _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
        _jacobianOplusXj(0,2) = y/z *fx;
        _jacobianOplusXj(0,3) = -1./z *fx;
        _jacobianOplusXj(0,4) = 0;
        _jacobianOplusXj(0,5) = x/z_2 *fx;
        
        _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
        _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
        _jacobianOplusXj(1,2) = -x/z *fy;
        _jacobianOplusXj(1,3) = 0;
        _jacobianOplusXj(1,4) = -1./z *fy;
        _jacobianOplusXj(1,5) = y/z_2 *fy;
        
        _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
        _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
        _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
        _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
        _jacobianOplusXj(2,4) = 0;
        _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;
    }
    
    Vector3 cam_project(const Vector3 & trans_xyz, const float &bf) const
    {
        const float invz = 1.0f/trans_xyz[2];
        Vector3 res;
        res[0] = trans_xyz[0]*invz*fx + cx;
        res[1] = trans_xyz[1]*invz*fy + cy;
        res[2] = res[0] - bf*invz;
        return res;
    }
    
    double fx, fy, cx, cy, bf;
};
#endif

#if 0 // now in g20
class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeSE3ProjectXYZOnlyPose(){}
    
    bool read(std::istream& is)
    {
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
            return true;
    }
    
    bool write(std::ostream& os) const
    {
        
        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }
        
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
            return os.good();
    }
    
    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Vector2 obs(_measurement);
        _error = obs-cam_project(v1->estimate().map(Xw));
    }
    
    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw))(2)>0.0;
    }
    
    
    virtual void linearizeOplus()
    {
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        Vector3 xyz_trans = vi->estimate().map(Xw);
        
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;
        
        _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
        _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
        _jacobianOplusXi(0,2) = y*invz *fx;
        _jacobianOplusXi(0,3) = -invz *fx;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = x*invz_2 *fx;
        
        _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
        _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
        _jacobianOplusXi(1,2) = -x*invz *fy;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -invz *fy;
        _jacobianOplusXi(1,5) = y*invz_2 *fy;
    }
    
    Vector2 cam_project(const Vector3 & trans_xyz) const
    {
        Vector2 proj = internal::project2d(trans_xyz);
        Vector2 res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }
    
    Vector3 Xw;
    double fx, fy, cx, cy;
};
#endif

#if 0 // now in g20
class  EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeStereoSE3ProjectXYZOnlyPose() : BaseUnaryEdge<3, Vector3, VertexSE3Expmap>() 
    {
        
    }
    
    bool read(std::istream& is)
    {
        for (int i=0; i<=3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
            return true;
    }
    
    bool write(std::ostream& os) const
    {
        
        for (int i=0; i<=3; i++){
            os << measurement()[i] << " ";
        }
        
        for (int i=0; i<=2; i++)
            for (int j=i; j<=2; j++){
                os << " " <<  information()(i,j);
            }
            return os.good();
    }
    
    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Vector3 obs(_measurement);
        _error = obs - cam_project(v1->estimate().map(Xw));
    }
    
    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw))(2)>0.0;
    }
    
    
    virtual void linearizeOplus()
    {
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        Vector3 xyz_trans = vi->estimate().map(Xw);
        
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;
        
        _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
        _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
        _jacobianOplusXi(0,2) = y*invz *fx;
        _jacobianOplusXi(0,3) = -invz *fx;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = x*invz_2 *fx;
        
        _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
        _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
        _jacobianOplusXi(1,2) = -x*invz *fy;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -invz *fy;
        _jacobianOplusXi(1,5) = y*invz_2 *fy;
        
        _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*y*invz_2;
        _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)+bf*x*invz_2;
        _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2);
        _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3);
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5)-bf*invz_2;
    }
    
    Vector3 cam_project(const Vector3 & trans_xyz) const
    {
        const float invz = 1.0f/trans_xyz[2];
        Vector3 res;
        res[0] = trans_xyz[0]*invz*fx + cx;
        res[1] = trans_xyz[1]*invz*fy + cy;
        res[2] = res[0] - bf*invz;
        return res;
    }
    
    Vector3 Xw;
    double fx, fy, cx, cy, bf;
};
#endif
    
class VertexSim3ExpmapTwoCam : public BaseVertex<7, Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3ExpmapTwoCam() : BaseVertex<7, Sim3>()
    {
        _marginalized=false;
        _fix_scale = false;
    }
    
    virtual bool read(std::istream& is)
    {
        Vector7 cam2world;
        for (int i=0; i<6; i++){
            is >> cam2world[i];
        }
        is >> cam2world[6];
        //    if (! is) {
        //      // if the scale is not specified we set it to 1;
        //      std::cerr << "!s";
        //      cam2world[6]=0.;
        //    }
        
        for (int i=0; i<2; i++)
        {
            is >> _focal_length1[i];
        }
        for (int i=0; i<2; i++)
        {
            is >> _principle_point1[i];
        }
        
        setEstimate(Sim3(cam2world).inverse());
        return true;
    }
    
    virtual bool write(std::ostream& os) const
    {
        Sim3 cam2world(estimate().inverse());
        Vector7 lv=cam2world.log();
        for (int i=0; i<7; i++){
            os << lv[i] << " ";
        }
        for (int i=0; i<2; i++)
        {
            os << _focal_length1[i] << " ";
        }
        for (int i=0; i<2; i++)
        {
            os << _principle_point1[i] << " ";
        }
        return os.good();
    }

    virtual void setToOriginImpl() {
        _estimate = Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<Vector7> update(const_cast<double*>(update_));

        if (_fix_scale)
        update[6] = 0;

        Sim3 s(update);
        setEstimate(s*estimate());
    }

    Vector2 _principle_point1, _principle_point2;
    Vector2 _focal_length1, _focal_length2;

    Vector2 cam_map1(const Vector2 & v) const
    {
        Vector2 res;
        res[0] = v[0]*_focal_length1[0] + _principle_point1[0];
        res[1] = v[1]*_focal_length1[1] + _principle_point1[1];
        return res;
    }

    Vector2 cam_map2(const Vector2 & v) const
    {
        Vector2 res;
        res[0] = v[0]*_focal_length2[0] + _principle_point2[0];
        res[1] = v[1]*_focal_length2[1] + _principle_point2[1];
        return res;
    }

    bool _fix_scale;
};

#if 0 // now in g20
class EdgeInverseSim3ProjectXYZ : public  BaseBinaryEdge<2, Vector2,  VertexSBAPointXYZ, VertexSim3ExpmapTwoCam>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ() : BaseBinaryEdge<2, Vector2, VertexSBAPointXYZ, VertexSim3ExpmapTwoCam>()
    {
        
    }
    
    virtual bool read(std::istream& is)
    {
        for (int i=0; i<2; i++)
        {
            is >> _measurement[i];
        }
        
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
            return true;
    }
    
    virtual bool write(std::ostream& os) const
    {
        for (int i=0; i<2; i++){
            os  << _measurement[i] << " ";
        }
        
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
            return os.good();
    }

    void computeError()
    {
      const VertexSim3ExpmapTwoCam* v1 = static_cast<const VertexSim3ExpmapTwoCam*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Vector2 obs(_measurement);
      _error = obs-v1->cam_map2(project(v1->estimate().inverse().map(v2->estimate())));
    }

   // virtual void linearizeOplus();

};
#endif

}

#endif // G2O_EXTENSIONS_H
