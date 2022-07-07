#include <fcpw/fcpw.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <igl/readDMAT.h>
#include <igl/parallel_for.h>
#include <igl/get_seconds.h>
#include <cstdio>

int main(int argc, char * argv[])
{
  const auto & tictoc = []()
  {
    static double t_start = igl::get_seconds();
    double diff = igl::get_seconds()-t_start;
    t_start += diff;
    return diff;
  };

  typedef Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> MatrixdX3R;
  MatrixdX3R V;
  // Use RowMajor so we can use direclty in setObjectTriangle
  Eigen::Matrix<int,Eigen::Dynamic,3,Eigen::RowMajor> F;

  igl::read_triangle_mesh(argv[1],V,F);

  {
    const int nVertices = V.rows();
    const int nTriangles = F.rows();
    using namespace fcpw;
    // initialize a 3d scene
    Scene<3> scene;
    
    // set the types of primitives the objects in the scene contain;
    // in this case, we have a single object consisting of only triangles
    scene.setObjectTypes({{PrimitiveType::Triangle}});
    
    // set the vertex and triangle count of the (0th) object
    scene.setObjectVertexCount(nVertices, 0);
    scene.setObjectTriangleCount(nTriangles, 0);
    
    // specify the vertex positions
    for (int i = 0; i < nVertices; i++) {
    	scene.setObjectVertex(Vector<3>(V(i,0),V(i,1),V(i,2)), i, 0);
    }
    
    // specify the triangle indices
    for (int i = 0; i < nTriangles; i++) {
    	scene.setObjectTriangle(&F(i,0), i, 0);
    }
    
    tictoc();
    // now that the geometry has been specified, build the acceleration structure
    scene.build(AggregateType::Bvh_SurfaceArea, true); // the second boolean argument enables vectorization
    printf("preprocess: %g\n",tictoc());
    
    // Generate a list of random query points in the bounding box
    MatrixdX3R Q;
    if(argc<=2 || !igl::readDMAT(argv[2],Q))
    {
      printf("generating random points\n");
      Q = MatrixdX3R::Random(1000000,3);
      const Eigen::RowVector3d Vmin = V.colwise().minCoeff();
      const Eigen::RowVector3d Vmax = V.colwise().maxCoeff();
      const Eigen::RowVector3d Vdiag = Vmax-Vmin;
      for(int q = 0;q<Q.rows();q++)
      {
        Q.row(q) = (Q.row(q).array()*0.5+0.5)*Vdiag.array() + Vmin.array();
      }
    }

    Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> UV(Q.rows(),2);
    Eigen::VectorXi I(Q.rows());
    tictoc();
    igl::parallel_for(Q.rows(),[&](const int q)//(int q = 0;q<Q.rows();q++)
    {
    // perform a closest point query
      Interaction<3> interaction;
      Vector<3> queryPoint(Q(q,0),Q(q,1),Q(q,2));
      scene.findClosestPoint(queryPoint, interaction);
      I(q) = interaction.primitiveIndex;
      UV(q,0) = interaction.uv[0];
      UV(q,1) = interaction.uv[1];
    });
    printf("query: %g\n",tictoc());
    igl::writeDMAT("Q.dmat",Q);
    igl::writeDMAT("I.dmat",I);
    igl::writeDMAT("UV.dmat",UV);
  }

}

