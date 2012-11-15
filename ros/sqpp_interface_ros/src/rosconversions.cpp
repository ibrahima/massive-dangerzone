#include <sqpp_interface_ros/rosconversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <boost/shared_ptr.hpp>
#include <openrave/kinbody.h>

OpenRAVE::KinBody moveitObjectToKinBody(collision_detection::CollisionWorld::Object object){
  std::vector<shapes::ShapeConstPtr> shapes = object.shapes_;
  std::vector<Eigen::Affine3d> poses; // Affine3D is a 4x4 affine transform matrix
  vector<OpenRAVE::KinBody::Link::GeometryInfo> geometries;
  for(int i = 0; i < shapes.size(); i++){
	shapes::ShapeConstPtr shape = shapes[i];
    OpenRAVE::KinBody::Link::GeometryInfo info;
    // Convert the Eigen::Affine3D pose into a RaveTransform (translation + quaternion)
    Eigen::Vector3d translation = poses[i].translation();
    Eigen::Matrix3d rot = poses[i].extractRotation();
    info._t.trans = RaveVector(translation[0], translation[1], translation[2]);
    RaveTransformMatrix rotm();
    rotm.rotfrommat(rot(0,0), rot(0,1), rot(0,2),
		    rot(1,0), rot(1,1), rot(1,2),
		    rot(2,0), rot(2,1), rot(2,2));
    info._t.rot = OpenRAVE::geometry::quatFromMatrix(rotm);

    // Fill in the geometry data (_vGeomData)
    switch(shape->type){
    case shapes::SPHERE: //for sphere it is radius
      info._type = OpenRAVE::GT_Sphere;
	  boost::shared_ptr<shapes::Sphere> sph = boost::dynamic_pointer_cast<shapes::Sphere>(shape);
      info._vGeomData.x = sph->radius;
      break;
    case shapes::CYLINDER: //for cylinder, first 2 values are radius and height
      boost::shared_ptr<shapes::Cylinder> cyl = boost::dynamic_pointer_cast<shapes::Cylinder>(shape);
      info._type = OpenRAVE::GT_Cylinder;
      info._vGeomData.x = cyl->radius;
      info._vGeomData.y = cyl->length;
      break;
    case shapes::BOX: //for boxes, first 3 values are extents
      boost::shared_ptr<shapes::Box> box = boost::dynamic_pointer_cast<shapes::Box>(shape);
      info._type = OpenRAVE::GT_Box;
      info._vGeomData.x = box->size[0];
      info._vGeomData.y = box->size[1];
      info._vGeomData. = box->size[2];
      break;
    case shapes::MESH:
      // Not actually sure if mesh and trimesh are the same thing.
      // Hopefully don't have to triangulate non-triangular meshes
      boost::shared_ptr<shapes::Sphere> sph = boost::dynamic_pointer_cast<shapes::Sphere>(shape);
      info._type = OpenRAVE::GT_Trimesh;
      continue; // TODO: Implement this and remove
      break;
    default:
      LOG_WARN("Unknown shape type");
      continue;
    }
    geometries.push_back(info);
  }
  //TODO: Set a name for the KinBody
  return OpenRAVE::KinBody::InitFromGeometries(geometries);
}

void importCollisionWorld(Environment::Ptr env, RaveInstance::Ptr rave, const collision_detection::CollisionWorldConstPtr world){
  std::vector<string> objectIds = world->getObjectIds();
  for(int i = 0; i < objectIds.size(); i++){
    collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(objectIds[i]);
	OpenRAVE::KinBody body = moveitObjectToKinBody(obj);
    env->add(RaveObject::Ptr(new RaveObject(rave, body, CONVEX_HULL, BulletConfig::kinematicPolicy == 0)));

  }
}
