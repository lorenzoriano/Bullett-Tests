#include "ColladaConverter.h"
#include <iostream>

#include "dom/domCOLLADA.h"
#include "btBulletDynamicsCommon.h"
#include <vector>
#include <iostream>

class GripperLoader : public ColladaConverter {

public:
    
    GripperLoader() :
        m_collisionConfiguration(new btDefaultCollisionConfiguration()),        
        m_dispatcher(new  btCollisionDispatcher(m_collisionConfiguration)),         
        m_broadphase(new btDbvtBroadphase()),        
        m_solver(new btSequentialImpulseConstraintSolver()),        
        m_world(new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration)),
        ColladaConverter(NULL)        
    {
        m_dynamicsWorld = m_world; 
        m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,
                                                  BOX_SHAPE_PROXYTYPE,
                                                  m_collisionConfiguration->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE,CONVEX_SHAPE_PROXYTYPE));
            
    }
    
    virtual ~GripperLoader() {
        delete m_world;
        delete m_solver;
        delete m_broadphase;
        delete m_dispatcher;
        delete m_collisionConfiguration;
    }
    
    std::vector<btVector3> shape_from_geometry(const char* nodeid) 
    {
        domLibrary_geometries* geomLib = getDefaultGeomLib ();
        domGeometry_Array& geometryArray = geomLib->getGeometry_array ();
        std::vector<btVector3> vertices;
        
        domGeometry* geom = findGeometry(nodeid);
        if (geom == NULL) {
            std::cerr<<"No geometry found with id: "<<nodeid<<"\n";
            return vertices;
        }
        std::cout<<"name: "<<geom->getName()<<"\n";
        domMesh* mesh = geom->getMesh();
        
        const domVerticesRef vertsRef = mesh->getVertices();
        int numInputs = vertsRef->getInput_array().getCount();
        for (int i=0;i<numInputs;i++)
        {
            domInputLocalRef localRef = vertsRef->getInput_array()[i];
            daeString str = localRef->getSemantic();
            if ( !strcmp(str,"POSITION"))
            {
                const domURIFragmentType& frag = localRef->getSource();
                
                daeElementConstRef constElem = frag.getElement();
                
                const domSourceRef node = *(const domSourceRef*)&constElem;
                const domFloat_arrayRef flArray = node->getFloat_array();
                if (flArray)
                {
                    int numElem = flArray->getCount();
                    const domListOfFloats& listFloats = flArray->getValue();
                    
                    for (int k=0;k+2<numElem;k+=3)
                    {
                        domFloat fl0 = listFloats.get(k);
                        domFloat fl1 = listFloats.get(k+1);
                        domFloat fl2 = listFloats.get(k+2);                        
                        vertices.push_back(btVector3(fl0,fl1,fl2)*m_unitMeterScaling);
                    }
                    
                }
            }
            
        }
        std::cout<<"I've got "<<vertices.size()<<" vertices for shape "<<nodeid<<"\n";
        return vertices;        
    }
    
    btCompoundShape* create_gripper(std::vector<btVector3> palm,
                                      std::vector<btVector3> finger,
                                      std::vector<btVector3> finger_tip) {
        
        btTransform T1, T2;
        btTransform T_proximal, T_distal;
        float angle = 0.541; //closed gripper
//         float angle = 0.021; //open gripper ??
        
        std::cout<<"WARNING: Enlarging the shape!\n";
        float factor = 10.;
        btVector3 scale(factor,factor,factor);
        
//         btVector3 scale(1,1,1);
        
        btCompoundShape* shape = createCompoundShape();
        
        //palm
        btConvexHullShape* palm_j = createConvexHullShape();
        for (unsigned int i=0; i<palm.size(); i++) {
            btVector3 point = palm[i];
            palm_j->addPoint(point);            
        }
        palm_j->setLocalScaling(scale);
        shape->addChildShape(btTransform::getIdentity(), palm_j);
        
        //first finger
        T1.setOrigin(btVector3(0.07691, 0.01, 0));
//         T1.setOrigin(btVector3(10.07691, 0.01, 0));
        T1.setRotation(btQuaternion(btVector3(0,0,1),  angle));
        T2.setOrigin(btVector3(0.09137, 0.00495, 0));
        T2.setRotation(btQuaternion(btVector3(0,0,1), -angle));
        T_proximal = T1;
        T_distal = T1 * T2;
        
        btConvexHullShape* first_finger = createConvexHullShape();
        for (unsigned int i=0; i<finger.size(); i++) {
            btVector3 point = finger[i];
            first_finger->addPoint(point);            
        }
        first_finger->setLocalScaling(scale);
        shape->addChildShape(T_proximal, first_finger);
        
        btConvexHullShape* first_finger_tip = createConvexHullShape();
        for (unsigned int i=0; i<finger_tip.size(); i++) {
            btVector3 point = finger_tip[i];
            first_finger_tip->addPoint(point);            
        }
        first_finger_tip->setLocalScaling(scale);
//         shape->addChildShape(T_distal, first_finger_tip);
        
        
        //second finger        
        T1.setOrigin(btVector3(0.07691, -0.01, 0));
        T1.setRotation(btQuaternion(btVector3(1,0,0), M_PI)*btQuaternion(btVector3(0,0,1),  angle));
        T2.setOrigin(btVector3(0.09137, 0.00495, 0));
        T2.setRotation(btQuaternion(btVector3(0,0,1), -angle));
        T_proximal = T1;
        T_distal = T1 * T2;
        
        btConvexHullShape* second_finger = createConvexHullShape();
        for (unsigned int i=0; i<finger.size(); i++) {
            btVector3 point = finger[i];
            second_finger->addPoint(point);            
        }
        second_finger->setLocalScaling(scale);
//         shape->addChildShape(T_proximal, second_finger);
        
        btConvexHullShape* second_finger_tip = createConvexHullShape();
        for (unsigned int i=0; i<finger_tip.size(); i++) {
            btVector3 point = finger_tip[i];
            second_finger_tip->addPoint(point);            
        }
        second_finger_tip->setLocalScaling(scale);
//         shape->addChildShape(T_distal, second_finger_tip);
     
        return shape;
    }
    
    void print_shape_info(btCompoundShape* c_shape) {
        int numshapes = c_shape->getNumChildShapes();
        std::cout<<"Number of children: "<<numshapes<<"\n";
        for (int i=0; i<numshapes; i++) {
            btConvexHullShape* shape = dynamic_cast<btConvexHullShape*>(c_shape->getChildShape(i));
            if (shape == NULL) {
                std::cerr<<"Error converting shape "<<i<<"\n";
                continue;
            }                
            std::cout<<"Shape "<<i<<" number of vertices: "<<shape->getNumVertices()<<"\n";
        }
        
    }
    
private:
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher* m_dispatcher;
    btDbvtBroadphase* m_broadphase;
    btSequentialImpulseConstraintSolver* m_solver;
    btDiscreteDynamicsWorld* m_world;

};

int main(int argc, char** argv) {
    
    if (argc != 4) {
        std::cerr<<"Give me the filenames!!!\n";
        return 1;
    }
    
    char* palm_filename = argv[1];
    char* finger_filename = argv[2];
    char* fingertip_filename = argv[3];
    
    std::vector< btVector3 > palm;      
    {
        GripperLoader loader_palm;
        std::cout<<"Loading: "<<palm_filename<<"\n";
        if (!loader_palm.load(palm_filename))
            return 1;
        palm = loader_palm.shape_from_geometry("gripper_palm");
    }    
    std::vector< btVector3 > finger;    
    {
        GripperLoader loader_finger;
        std::cout<<"Loading: "<<finger_filename<<"\n";
        if (!loader_finger.load(finger_filename))
            return 1;
        finger = loader_finger.shape_from_geometry("l_finger");
    }
    std::vector< btVector3 > finger_tip;
    {
        GripperLoader loader_fingertip;    
        std::cout<<"Loading: "<<fingertip_filename<<"\n";
        if (!loader_fingertip.load(fingertip_filename))
            return 1;
        finger_tip = loader_fingertip.shape_from_geometry("l_finger_tip");
    }
    
    GripperLoader loader;
    btCompoundShape* shape = loader.create_gripper(palm, finger, finger_tip);
    std::cout<<"\n";
    loader.print_shape_info(shape);
    std::cout<<"\n";
//     std::cout<<"Got a shape with "<<shape->getNumPoints()<<" elements\n";
    
    //serializing
    std::cout<<"serializing to gripper.bullet\n";
    btDefaultSerializer* serializer = new btDefaultSerializer();
    serializer->startSerialization();
    serializer->registerNameForPointer(shape, "gripper");
    shape->serializeSingleShape(serializer);
    serializer->finishSerialization();
    
    FILE* file = fopen("objects/gripper.bullet", "wb");
    fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(), 1, file);
    fclose(file);
    
    delete serializer;
    
}