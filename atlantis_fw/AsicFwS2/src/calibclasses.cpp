#include "log.h"
#include "calibclasses.h"
#include "operator_new.h"


#define SLOGF_ID                SLOG_ID_AF


namespace ltaf {

void CalibData::setCameraParameters( const Matrix3x3f & K,
                                     const Matrix3x3f & R,
                                     const Vec3f & translation )
{
    _K = K;
    _R = R;
    _T = translation;
}

void CalibData::scale( float s )
{
    if( s == 1.0f ) { return; }

    assert( s > 0 ); //, "Scale has to be a positive value." );

    _K( 0, 0 ) *= s;
    _K( 1, 1 ) *= s;
    _K( 1, 2 ) *= s;
    _K( 0, 2 ) *= s;
}

void CalibData::shift( int delta_x, int delta_y )
{
    _K( 0, 2 ) -= delta_x;
    _K( 1, 2 ) -= delta_y;
}


EpiField::EpiField( const CalibData & ref, const CalibData & src )
{
    setup( ref, src );
}

void EpiField::setup( const CalibData & ref, const CalibData & src )
{
    /// Skwe matrix for cross product
    auto skew = []( const Vec3f & p ) ->Matrix3x3f
    {
        return Matrix3x3f( {0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0} );
    };

    Matrix3x3f R_rel   = src.R() * transpose( ref.R() );
    Vec3f T_rel = R_rel *  ref.translation() - src.translation();
    Vec3f epipole = ref.K() * transpose( R_rel ) * T_rel;
    _F = transpose( inverse( src.K() ) ) * R_rel * transpose( ref.K() ) * skew( epipole );
}



static Matrix4x4d CreateExtrinsicMatrix( const Matrix3x3f & rot, const Vec3f & translation )
{
    Matrix4x4d extrinsic( rot );
    extrinsic( 0, 3 ) = translation.x();
    extrinsic( 1, 3 ) = translation.y();
    extrinsic( 2, 3 ) = translation.z();
    return extrinsic;
}


WarpField::WarpField( const CalibData & ref,
                      const CalibData & src ){
        Matrix4x4d e_ref = CreateExtrinsicMatrix( ref.R(), ref.translation() );
        Matrix4x4d e_src = CreateExtrinsicMatrix( src.R(), src.translation() );
        Matrix4x4d k_ref( ref.K() ); // expand with 0* and diagonal is 1.
        Matrix4x4d k_src( src.K() );
        _mat = Matrix4x4f( transpose( ( k_src * e_src ) * inverse( ( k_ref * e_ref ) ) ) );
}

/// 3D projection from (x,y,z) to (x',y',z')
Vec3f WarpField::project3D( float x, float y, float depth ) const
{
    const Vec4f out = Vec4f( x * depth, y * depth, depth, 1.0f ) * _mat;
    const float inv_z = 1.0f / out.z();
    return Vec3f( out.x() * inv_z, out.y() * inv_z, out.z() );
}


float CalibUtils::depthGivenCorrespondence( const EpiField & epf,
                                   const CalibData & ref, const CalibData & src,
                                   float x, float y,
                                   float x_src, float y_src )
{
    SLOGF(SLOG_DEBUG, "depthGivenCorrespondence. x: %f, y: %f, x_src: %f, y_src: %f", x, y, x_src, y_src);

    // We first project the source point to the epipolar line
    // induced by (x,y) on current image. Then compute the world coordinate (a1, a2)
    // and the baseline (b1, b2), and then use Laws of sines to reconstruct
    // the distance between object and camera 1.

    // We first project / correct the source point to the epipolar line
    // induced by ref image (x,y) onto the src image.
    Vec2f src_proj = epf( x, y ).closest( Vec2f( x_src, y_src ) );
    SLOGF(SLOG_DEBUG, "depthGivenCorrespondence. src_proj.x: %f, src_proj.y: %f", src_proj.x(), src_proj.y());

    // a1 is the vector from ref camera center to (x y 1) on the image plane
    Vec3f a1 = inverse( ref.K() ) * Vec3f( x, y, 1 );
    // a2 is the vector from src camera center to (x_src y_src 1) on the image plane
    Vec3f a2 = inverse( src.K() ) * Vec3f( src_proj.x(), src_proj.y(), 1 );

    // b1 is the vector from ref to src camera center, b2 is the other way round.
    Matrix3x3f refR_srcR_T = ref.R() * transpose( src.R() ); // ref R * (src R)^T
    Vec3f b1 =  ref.translation() - refR_srcR_T * src.translation();
    Vec3f b2 =  src.translation() - transpose( refR_srcR_T ) * ref.translation();

    float a1a1 = a1.dot( a1 );
    float b1b1 = b1.dot( b1 );

    // theta_i is the angle between a_i and b_i
    float cos_theta_1 = a1.dot( b1 ) / std::sqrt( a1a1 * b1b1 );
    float cos_theta_2 = a2.dot( b2 ) / std::sqrt( a2.dot( a2 ) * b2.dot( b2 ) );
    float sin_theta_1 = std::sqrt( 1 - cos_theta_1 * cos_theta_1 );
    float sin_theta_2 = std::sqrt( 1 - cos_theta_2 * cos_theta_2 );

    // phi is the angle between a1 and 2.
    float sin_phi =  sin_theta_1 * cos_theta_2 + sin_theta_2 * cos_theta_1;
    float s_theta2_phi = std::abs(sin_theta_2 / sin_phi);
    // world point to ref camera center dist / a1 length is equal to
    // world point depth / 1.
    // depth = dist / a1 length
    // where dist / sin theta_2 = b1 length / sin phi.

    float d = s_theta2_phi * std::sqrt(b1b1 / a1a1);

    SLOGF(SLOG_DEBUG, "depthGivenCorrespondence. d: %f, sin_phi: %f", d, sin_phi);

    return sin_phi > 0 ? d : 0.0f;
}

// compute depth given point to point correspondence
float CalibUtils::depthGivenCorrespondence( const CalibData & ref,
                                       const CalibData & src,
                                       float x, float y,
                                       float x_src, float y_src )
{
    // We first project the source point to the epipolar line
    // induced by (x,y) on current image. Then compute the world coordinate (a1, a2)
    // and the baseline (b1, b2), and then use Laws of sines to reconstruct
    // the distance between object and camera 1.

    EpiField epf( ref, src );
    return depthGivenCorrespondence(epf, ref, src, x, y, x_src, y_src);
}

// ----------------- calib data manager ---------------------

CalibDataManager::CalibDataManager(const Ltpb__LightHeader& lh) {
    if (!parseGeometricCalibData(lh)) {
        SLOGF(SLOG_ERROR, "Parsing calibration protobuf failed...");
    }
}

void CalibDataManager::setMirrorTypes(ModuleName m, MirrorType t){
    _cam_mirror_types[m] = t;
}

//whether we have calibration data for this module
bool CalibDataManager::isModuleCalibrated(ModuleName m) const{
    return _cam_params[m].get()!=nullptr;
}

bool CalibDataManager::isModuleMirrorMoveable(ModuleName m) const {
    return _cam_mirror_types[m] == MirrorType::Moveable;
}

const CalibDataBase * CalibDataManager::getCamParams(ModuleName m) const {
    return _cam_params[m].get();
}


void CalibDataManager::parseMirrorSystem(const Ltpb__MirrorSystem * m, MirrorSysParam & msp){
    msp = MirrorSysParam({
        Matrix3x3f{   m->real_camera_orientation->x00,m->real_camera_orientation->x01,m->real_camera_orientation->x02,
                      m->real_camera_orientation->x10,m->real_camera_orientation->x11,m->real_camera_orientation->x12,
                      m->real_camera_orientation->x20,m->real_camera_orientation->x21,m->real_camera_orientation->x22 },
        Vec3f{m->real_camera_location->x, m->real_camera_location->y, m->real_camera_location->z },
        Vec3f{m->point_on_rotation_axis->x, m->point_on_rotation_axis->y, m->point_on_rotation_axis->z},
        Vec3f{m->rotation_axis->x, m->rotation_axis->y, m->rotation_axis->z},
        Vec3f{m->mirror_normal_at_zero_degrees->x, m->mirror_normal_at_zero_degrees->y, m->mirror_normal_at_zero_degrees->z},
        m->distance_mirror_plane_to_point_on_rotation_axis,
        (m->flip_img_around_x)>0, // protobuf-c-boolean is int
        Vec2f{m->mirror_angle_range->min_val, m->mirror_angle_range->max_val}
    });
}


void CalibDataManager::parseMirrorActuatorMapping(const Ltpb__MirrorActuatorMapping * m, MirrorActuatorMapping & mam) {
    mam = MirrorActuatorMapping({
        m->actuator_length_offset, m->actuator_length_scale, m->mirror_angle_offset, m->mirror_angle_scale, 
        m->quadratic_model->model_coeffs[3], m->quadratic_model->model_coeffs[4], m->quadratic_model->model_coeffs[5], 
        m->quadratic_model->use_rplus_for_left_segment > 0 // int to bool
    });
}

bool CalibDataManager::parseGeometricCalibData(const Ltpb__LightHeader& lh) {

    // parse the data
    SLOGF(SLOG_DEBUG, "Found %i factory module calibration", int(lh.n_module_calibration));
    for (std::size_t i = 0; i < lh.n_module_calibration; ++i) {
        
        Ltpb__FactoryModuleCalibration * fmc = lh.module_calibration[i];
        ModuleName curr_module_name = static_cast<ModuleName>(fmc->camera_id);
        SLOGF(SLOG_DEBUG, "Current camera id = %i", int(fmc->camera_id));

        Ltpb__GeometricCalibration * geometry = fmc->geometry;

        bool calib_w_mirror = false;

        // --------------- check mirror type ------------------
        Ltpb__GeometricCalibration__MirrorType mirrortype = geometry->mirror_type;
        if (    mirrortype == LTPB__GEOMETRIC_CALIBRATION__MIRROR_TYPE__NONE  ||
                mirrortype == LTPB__GEOMETRIC_CALIBRATION__MIRROR_TYPE__GLUED ){
            SLOGF(SLOG_DEBUG, "Current module has no moveable mirror");
            _cam_mirror_types[curr_module_name] = MirrorType::NotMoveable;
        }
        else if (mirrortype == LTPB__GEOMETRIC_CALIBRATION__MIRROR_TYPE__MOVABLE){
            SLOGF(SLOG_DEBUG, "Current module mirror is moveable");
            calib_w_mirror = true;
            _cam_mirror_types[curr_module_name] = MirrorType::Moveable;
        }
        else{
            SLOGF(SLOG_ERROR, "Unknown mirror type: %d", int(mirrortype));
            return false;
        }

        // ---------------- get number of focus -------------------
        SLOGF(SLOG_DEBUG, "n_per_focus_calibration = %i", int(geometry->n_per_focus_calibration));

        // use the first one? todo: if more than 1, check which one to use for now
        Ltpb__GeometricCalibration__CalibrationFocusBundle * gcfb = geometry->per_focus_calibration[0];

        float fd = gcfb->focus_distance;
        SLOGF(SLOG_DEBUG, "Choosing calibration data with focus distance = %f", fd);

        Ltpb__GeometricCalibration__Intrinsics * intrinsics = gcfb->intrinsics;

        // --------------- get K mat ------------------
        Ltpb__Matrix3x3F * k_mat = intrinsics->k_mat;
        Matrix3x3f K({ k_mat->x00, k_mat->x01, k_mat->x02, k_mat->x10, k_mat->x11, k_mat->x12, k_mat->x20, k_mat->x21, k_mat->x22 });

        Ltpb__GeometricCalibration__Extrinsics * extrinsics = gcfb->extrinsics;
        if (extrinsics == nullptr) {
            extrinsics = geometry->per_focus_calibration[geometry->n_per_focus_calibration-1]->extrinsics;
        }

        // ----------------- get extrinsics ------------------
        if (calib_w_mirror) { // use mirror system params and actuator mapping
            Ltpb__MirrorSystem *mirror_system = extrinsics->moveable_mirror->mirror_system;
            Ltpb__MirrorActuatorMapping *mirror_actuator_mapping = extrinsics->moveable_mirror->mirror_actuator_mapping;
            if ((mirror_system != nullptr) && (mirror_actuator_mapping != nullptr)) {
                MirrorSysParam msp;
                parseMirrorSystem(mirror_system, msp);

                MirrorActuatorMapping mam;
                parseMirrorActuatorMapping(mirror_actuator_mapping, mam);

                _cam_params[curr_module_name].reset(new(Heap::DDR) CalibDataWMirror( K, msp, mam ));
            }
        } else { // normal R and t matrices
            Ltpb__GeometricCalibration__Extrinsics__CanonicalFormat * canonical = extrinsics->canonical;
            Ltpb__Matrix3x3F *rotation = canonical->rotation;
            Matrix3x3f R{ rotation->x00, rotation->x01, rotation->x02, rotation->x10, rotation->x11, rotation->x12, rotation->x20, rotation->x21, rotation->x22 };
            Ltpb__Point3F * translation = canonical->translation;
            Vec3f t{translation->x, translation->y, translation->z};
            _cam_params[curr_module_name].reset(new(Heap::DDR) CalibData(K, R, t));
        }
    }
#if 0 // these logs cause later code failure for some still unknown issue - stack overflow??
    CalibData& ca1 = *static_cast<CalibData*>(_cam_params[A1].get());
    SLOGF(SLOG_DEBUG, "A1 calib K: %f, %f, %f, %f, %f, %f, %f, %f, %f",
            ca1.K()(0,0), ca1.K()(0,1), ca1.K()(0,2),
            ca1.K()(1,0), ca1.K()(1,1), ca1.K()(1,2),
            ca1.K()(2,0), ca1.K()(2,1), ca1.K()(2,2));
    SLOGF(SLOG_DEBUG, "A1 calib R: %f, %f, %f, %f, %f, %f, %f, %f, %f",
            ca1.R()(0,0), ca1.R()(0,1), ca1.R()(0,2),
            ca1.R()(1,0), ca1.R()(1,1), ca1.R()(1,2),
            ca1.R()(2,0), ca1.R()(2,1), ca1.R()(2,2));
    SLOGF(SLOG_DEBUG, "A1 calib t: %f, %f, %f",
            ca1.translation().x(), ca1.translation().y(), ca1.translation().z());

    CalibData& ca5 = *static_cast<CalibData*>(_cam_params[A5].get());
    SLOGF(SLOG_DEBUG, "A5 calib K: %f, %f, %f, %f, %f, %f, %f, %f, %f",
            ca5.K()(0,0), ca5.K()(0,1), ca5.K()(0,2),
            ca5.K()(1,0), ca5.K()(1,1), ca5.K()(1,2),
            ca5.K()(2,0), ca5.K()(2,1), ca5.K()(2,2));
    SLOGF(SLOG_DEBUG, "A5 calib R: %f, %f, %f, %f, %f, %f, %f, %f, %f",
            ca5.R()(0,0), ca5.R()(0,1), ca5.R()(0,2),
            ca5.R()(1,0), ca5.R()(1,1), ca5.R()(1,2),
            ca5.R()(2,0), ca5.R()(2,1), ca5.R()(2,2));
    SLOGF(SLOG_DEBUG, "A5 calib t: %f, %f, %f",
            ca5.translation().x(), ca5.translation().y(), ca5.translation().z());
#endif
    return true;
}


void CalibDataManager::setDefaultMirrorTypes(){
    //A1 - A5
    _cam_mirror_types[0] = MirrorType::NotMoveable;
    _cam_mirror_types[1] = MirrorType::NotMoveable;
    _cam_mirror_types[2] = MirrorType::NotMoveable;
    _cam_mirror_types[3] = MirrorType::NotMoveable; 
    _cam_mirror_types[4] = MirrorType::NotMoveable;
    //B1 - B5
    _cam_mirror_types[5] = MirrorType::Moveable;
    _cam_mirror_types[6] = MirrorType::Moveable;
    _cam_mirror_types[7] = MirrorType::Moveable;
    _cam_mirror_types[8] = MirrorType::NotMoveable; //B4 glued
    _cam_mirror_types[9] = MirrorType::Moveable;
    //C1 - C6
    _cam_mirror_types[10] = MirrorType::Moveable;
    _cam_mirror_types[11] = MirrorType::Moveable;
    _cam_mirror_types[12] = MirrorType::Moveable;
    _cam_mirror_types[13] = MirrorType::Moveable;
    _cam_mirror_types[14] = MirrorType::NotMoveable;    //C5 glued
    _cam_mirror_types[15] = MirrorType::Moveable;
}


//------for hardcording calib --------
#define USE_P2_17


#if defined(USE_P2_9)
void CalibDataManager::hardCodeCalib(){
    // set everything to null first
    for (int i = 0; i<kNumModules; ++i){
        _cam_params[i].reset();
    }

    _cam_params[ModuleName::A1].reset( new(Heap::DDR) CalibData(Matrix3x3f({3295.0591148450653,0,2079.270095486714,0,3292.8691004956404,1560.1650789545195,0,0,1}),
                                            Matrix3x3f({0.9999999999999999,-2.3816433467529536e-20,4.375413466415473e-19,-2.3816433467529536e-20,1.0000000000000003,-5.407661426330766e-19,4.375413466415473e-19,-5.407661426330766e-19,1}),
                                            Vec3f({0,0,0})) );

    _cam_params[ModuleName::A2].reset( new(Heap::DDR) CalibData(Matrix3x3f({3292.662800728938,0,2078.7406070300717,0,3290.223434014899,1563.8775673883044,0,0,1}),
                                            Matrix3x3f({0.9999811814247545,0.0011315950069566996,-0.006029617657251501,-0.0011848747692272814,0.9999602234838719,-0.008840108702169737,0.006019374397199182,0.00884708668574911,0.9999427464555364}),
                                            Vec3f({29.49858846344386,26.503786194567483,3.8028354073386216})) );

//A4 stuck, no calibration data available.

    _cam_params[ModuleName::A3].reset( new(Heap::DDR) CalibData(Matrix3x3f({3313.6273113454727,0,2071.67367529091,0,3313.3143142018857,1544.2347471164574,0,0,1}),
                                            Matrix3x3f({0.9999987383721348,0.0014672622200696548,-0.0006086014427068554,-0.001471154059734803,0.9999781528069575,-0.006444347485794019,0.0005991325988747439,0.0064452347019088589,0.9999790497754272}),
                                            Vec3f({-10.019090578826673,26.520590074318777,4.52701528585639})) );

    _cam_params[ModuleName::A5].reset( new(Heap::DDR) CalibData(Matrix3x3f({3291.0635576697628,0,2141.044986965821,0,3294.0626079852855,1553.8797618889014,0,0,1}),
                                            Matrix3x3f({0.9999589422747642,0.004341889206131372,-0.007953726350340272,-0.004347768951795986,0.9999902876893729,-0.0007221025322346437,0.007950513812088885,0.0007566538488243728,0.9999681078939852}),
                                            Vec3f({42.10074769960174,3.8443510765515525,0.9321697446310342})) );

    _cam_params[ModuleName::B4].reset( new(Heap::DDR) CalibData(Matrix3x3f({8266.011118041928,0,2120.0862569026162,0,8299.797770250441,1624.3782703077686,0,0,1}),
                                            Matrix3x3f({0.99722741660368,-0.024544865893738507,-0.07024976250642283,0.021959583412439259,0.9990613093983918,-0.037340015526553,0.07110032538816219,0.03569383170002191,0.9968303236299887}),
                                            Vec3f({6.299786067286529,-16.573715979308394,33.081797314481978})) );


// B2
    _cam_params[ModuleName::B2].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8282.306922904852,0,2126.069870006372,0,8313.334914265235,1684.4681360942177,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{0.35902530983180727,0.4644519275409676,-0.8095586661284373,-0.4951154128580389,-0.6405044588869528,-0.5870389817519321,-0.7911773220022098,0.6115868255198206,-3.943384063731514e-9},
                     Vec3f{3.0433017274683729,-8.723102708377145,-19.693766500410278},
                     Vec3f{-9.257113888455056,-16.95579212058076,-10.34046237360415},
                     Vec3f{0.49610134984994716,-0.861758302992775,-0.10609466480494355},
                     Vec3f{-0.7816876221803366,-0.6235675196751278,-0.011314138776807042},
                     9.173385620117188f, 
                     true, 
                     Vec2f{39.05859968494464,42.13551163101412}
                 }, // mirror sys param
                MirrorActuatorMapping({443.3333333333333, 92.11586906355133,40.63525138761976,1.5398777621144325,
                                                -0.012551557522439664,0.9990424741648483,0.008367705014958026, true})
                ) );
                                       


// B3 is probably not well calibrated given the mirror movement range.
    _cam_params[ModuleName::B3].reset( new(Heap::DDR) CalibDataWMirror(
                Matrix3x3f{8256.814045307285,0,2182.607354070612,0,8284.332223450714,1679.218343799827,0,0,1}, 
                MirrorSysParam{   
                    Matrix3x3f{0.37533429180019087,-0.49805673862472796,-0.7817056060367922,0.4704594576427148,-0.624284833408494,0.623647613232546,-0.7986188503368623,-0.6018371306978559,-2.7002884372961945e-9},
                    Vec3f{-43.420606563817099,77.79990800085051,-91.84131049208833},
                    Vec3f{-37.495629583599058,74.20560224514296,-103.57297135387853},
                    Vec3f{-0.58026344223544,-0.8132449211597401,-0.04389801604818028},
                    Vec3f{-0.8099616372228859,0.586457715964707,-0.005430802213442899},
                    -47.38914108276367, 
                    true, 
                    Vec2f{42.33553405632188,42.49524020951211}
                }, // mirror sys param
                MirrorActuatorMapping{539.3333333333334, 3.055050463303893, 42.40638788351465,0.08136014571718942,
                                              -0.005477918887415739,1.0017252745521648,0.0036519459249559195, true}
                ) );

//B5
    _cam_params[ModuleName::B5].reset( new(Heap::DDR) CalibDataWMirror(
                Matrix3x3f{8253.018231946164,0,2067.820266354399,0,8260.310544194503,1569.025940062613,0,0,1}, 
                MirrorSysParam{
                   Matrix3x3f{-0.35702511296356956,-0.4636726051390165,0.8108888850865749,0.49471569176905719,0.6424929324703981,0.5852001504127045,-0.792331655956528,0.6100906055408379,1.8121704137286088e-9},
                        Vec3f{-45.513762811923538,23.307570608824823,-10.801205567561976},
                        Vec3f{-47.4283540845383,21.967579358041726,-18.55051558814425},
                        Vec3f{-0.6030252445335417,0.7976454264036477,0.011060207620370695},
                        Vec3f{0.8164065404110127,0.5774769966018447,0.0008241176097991389},
                        9.917757034301758, 
                        false, 
                        Vec2f{42.385095204988449,44.451176131153278}
                }, // mirror sys param
                MirrorActuatorMapping{444.3333333333333, 92.04527871288855,43.435002111555778,1.0334534479660996,
                                                -0.00538141028115564,0.9997320073549088,0.0035876068541013039, true}
                ) );

// to do: C calib data
                                            
}


#elif defined(USE_P2_20)

void CalibDataManager::hardCodeCalib(){
    // set everything to null first
    for (int i = 0; i<kNumModules; ++i){
        _cam_params[i].reset();
    }

    _cam_params[ModuleName::A1].reset( new(Heap::DDR) CalibData(Matrix3x3f({3280.5305426947125,0,2058.4341344648333,0,3276.9920888991874,1570.480280606213,0,0,1}),
                                            Matrix3x3f({0.9999999999999999,-3.748872363731547e-21,2.14886256680855e-19,-3.748872363731547e-21,0.9999999999999999,5.686272337510806e-19,2.14886256680855e-19,5.686272337510806e-19,1}),
                                            Vec3f({0,0,0})) );

    


    _cam_params[ModuleName::A2].reset( new(Heap::DDR) CalibData(Matrix3x3f({3304.8448314723137,0,2125.4828360403603,0,3304.8033227972273,1497.6494449744114,0,0,1}),
                                            Matrix3x3f({0.9999982149649729,0.001261901856627429,-0.001406296758191717,-0.0012691142085501785,0.9999859869537083,-0.00513957637779089,0.0013997914107167314,0.00514135195466429,0.9999858034412712}),
                                            Vec3f({26.850456053996465,23.155008470843357,2.451064213071837})) );

    _cam_params[ModuleName::A3].reset( new(Heap::DDR) CalibData(Matrix3x3f({3282.4003167172496,0,2068.242759586855,0,3279.714744414127,1575.9144575150526,0,0,1}),
                                            Matrix3x3f({0.9999970223716608,0.0022106388076789395,-0.001033597539718694,-0.0022219079560536348,0.9999366784833829,-0.011031869680819772,0.001009144611517229,0.011034133390608832,0.9999386128795463}),
                                            Vec3f({-8.723877086190978,23.67454003577845,-0.745511171951828})) );

    _cam_params[ModuleName::A4].reset( new(Heap::DDR) CalibData(Matrix3x3f({3275.915659596053,0,2046.8982075377758,0,3273.4084343421737,1558.3809500333957,0,0,1}),
                                            Matrix3x3f({0.9999702671665456,0.007698306272300902,0.00044817787217443886,-0.007697204321277828,0.999967470440399,-0.0024106237089937,-0.00046672101276473775,0.0024071023176660377,0.9999969940104463}),
                                            Vec3f({-24.500993753695817,-21.6141971758371,-5.098491338177205})) );


    _cam_params[ModuleName::A5].reset( new(Heap::DDR) CalibData(Matrix3x3f({3304.1046029307777,0,2069.6957686494256,0,3307.433978053652,1568.407095889419,0,0,1}),
                                            Matrix3x3f({0.9999847359836309,0.0031492576152972768,0.004539821166113563,-0.003100807323117871,0.999938584609053,-0.01064011296877998,-0.0045730508080163699,0.010625873447203928,0.9999330867712055}),
                                            Vec3f({42.11328364949198,-2.6510973441808867,3.607048367644486})) );

    _cam_params[ModuleName::B4].reset( new(Heap::DDR) CalibData(Matrix3x3f({8229.28539453575,0,2148.9274933845,0,8261.94410262734,1678.487052433296,0,0,1}),
                                            Matrix3x3f({0.9995873830352314,-0.017279216446002538,-0.022945421238008025,0.016841886507004704,0.9996753743210273,-0.01911797151894498,0.02326831613294368,0.018723638939218358,0.9995539059046347}),
                                            Vec3f({5.504509634741404,-15.753686243478086,25.854598108610554})) );


// B1
    _cam_params[ModuleName::B1].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8275.313632245749,0,2094.5997431002756,0,8286.984429043388,1544.2275981149978,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{-0.3641656880387787,0.49425524760685288,0.7893637322985719,-0.4682311373266157,0.6354956364178716,-0.6139258083283912,-0.8050732598298537,-0.5931753925332144,-8.96515106596496e-9},
                     Vec3f{26.47229190174795,5.158908511759591,-35.07128645824435},
                     Vec3f{28.47067823535999,3.637715106803811,-33.41342208940389},
                     Vec3f{0.6214301250421991,0.7828662154992272,-0.030742288788487654},
                     Vec3f{0.7979061100016143,-0.6027780078129306,-0.0021243632378838115},
                     -1.3221596479415894, 
                     false, 
                     Vec2f{37.26603296696898,46.99395319979643}
                 }, // mirror sys param
                MirrorActuatorMapping({ 453.75, 332.4649104291559, 42.27017752944356, 4.213756016889902,
                                               -0.0011437340034409093,0.9999093375537627,0.0008578005025791174, true})
                ) );

// B2
    _cam_params[ModuleName::B2].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8271.513642064405,0,2171.1273106952412,0,8300.886741675955,1632.373969307733,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{0.3568184935347023,0.4688460300048786,-0.8079999769927534,-0.4893378988461424,-0.6429715262537934,-0.5891825160166498,-0.7957568620027528,0.605616228791412,-1.7864398849098962e-10},
                     Vec3f{-21.159360554445614,-26.222093569425835,-42.73208500275925},
                     Vec3f{-24.47737463429766,-28.741867328001658,-38.72128215535967},
                     Vec3f{0.5942594595924562,-0.8041584471080813,-0.013597302291322136},
                     Vec3f{-0.8039515218521105,-0.5946948223961965,-0.00013684601562877842},
                     -5.778927803039551, 
                     true, 
                     Vec2f{35.663086993164167,45.31076020855335}
                 }, // mirror sys param
                MirrorActuatorMapping({444.5,315.5339390092081,40.48567517294649, 4.16355227533705,
                                                -0.005857171209437758,0.9999959776612998,0.004392878407078245, true})
                ) ); 

//B3 is missing for p2-20

// B5
    _cam_params[ModuleName::B2].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8238.008153704817,0,2029.3181668738032,0,8255.493325729016,1587.1413041816738,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{-0.36062486332429058,-0.4733771385674304,0.8036565140867387,0.48701345147175786,0.6392821683103748,0.595093444230359,-0.7954669107121316,0.6059970247138986,-1.047103626028445e-8},
                     Vec3f{-42.78451366088321,26.89612072257969,-12.512978606932265},
                     Vec3f{-36.595502311786109,31.470163821871613,-6.2522884437565999},
                     Vec3f{-0.5853839326147728,0.8106426264176462,-0.013571428486486714},
                     Vec3f{0.8036517779860654,0.5950986168243709,-0.0012066455803420567},
                     10.94692611694336, 
                     false, 
                     Vec2f{36.50045993643318,47.21181177090214}
                 }, // mirror sys param
                MirrorActuatorMapping({464.5,   329.9803024424337,   41.8921133561107, 4.610729865614154,
                                                0.007934248427417168,1.0001320962250342,-0.0059506863205628988, true})
                ) );
}

#elif defined(USE_P2_17)

void CalibDataManager::hardCodeCalib() {
    // set everything to null first
    for (int i = 0; i<kNumModules; ++i){
        _cam_params[i].reset();
    }
    _cam_params[ModuleName::A1].reset( new(Heap::DDR) CalibData(Matrix3x3f({3283.0178806848958,0,2067.6537966273186,0,3280.6893757352545,1578.1877773512508,0,0,1}),
                                            Matrix3x3f({1,-5.758690746490504e-21,-3.1788115867896309e-19,-5.758690746490504e-21,1,-1.2827678728194166e-19,-3.1788115867896309e-19,-1.2827678728194166e-19,1}),
                                            Vec3f({0,0,0})) );

    _cam_params[ModuleName::A2].reset( new(Heap::DDR) CalibData(Matrix3x3f({3288.40439677336,0,2060.366186836129,0,3286.0124276038546,1580.1577722277204,0,0,1}),
                                            Matrix3x3f({0.999992536045131,-0.003115561090423153,-0.0022849798947040479,0.003108431291599491,0.9999903102149502,-0.003117231321714926,0.0022946696783558079,0.003110105351835651,0.9999925308399896}),
                                            Vec3f({27.544676082731497,24.054302671695046,4.293053689571898})) );

    _cam_params[ModuleName::A3].reset( new(Heap::DDR) CalibData(Matrix3x3f({3283.9925702283035,0,2044.7456104113876,0,3281.1604400085794,1576.9265652924962,0,0,1}),
                                            Matrix3x3f({0.9999877558591814,0.004948227040131077,0.000056399268737690914,-0.004947164839137641,0.9999127234621246,-0.012250347693487839,-0.00011701184811270116,0.01224991868202635,0.9999249600847612}),
                                            Vec3f({-8.419114067217319,22.64080263740905,1.8599859541358175})) );
    _cam_params[ModuleName::A4].reset( new(Heap::DDR) CalibData(Matrix3x3f({3272.9707507103954,0,2058.174521191743,0,3271.4409417771963,1569.4878447691929,0,0,1}),
                                            Matrix3x3f({0.9999854308705849,-0.0012390067725664579,-0.0052538470465188,0.0012224069870125814,0.9999942550384939,-0.0031615836483592397,0.005257734086922112,0.00315511524749955,0.9999812005633137}),
                                            Vec3f({-24.784214131925994,-22.880831625046313,-3.524559324784576})) );
    _cam_params[ModuleName::A5].reset( new(Heap::DDR) CalibData(Matrix3x3f({3303.334581027952,0,2057.1596563099565,0,3306.0615289458306,1588.1707648379631,0,0,1}),
                                            Matrix3x3f({0.9999940192706713,-0.002709484380127736,0.002149445808129158,0.00272653897962734,0.9999645094124698,-0.007971568269232545,-0.002127770683323956,0.007977381141220862,0.9999659164101779}),
                                            Vec3f({41.85717528861697,0.2158910298621528,2.1832667916754846})) );
    _cam_params[ModuleName::B4].reset( new(Heap::DDR) CalibData(Matrix3x3f({8229.99722654244,0,2097.192877434885,0,8256.353502889702,1596.7833233634226,0,0,1}),
                                            Matrix3x3f({0.9992941887172542,-0.018254458054663124,-0.03283137458370294,0.01761810102232996,0.9996532768082757,-0.019568563637730957,0.03317720470883355,0.01897632545061068,0.9992693191327855}),
                                            Vec3f({5.352479176132089,-13.739072564681024,30.322880018517997})) );



//B1
    _cam_params[ModuleName::B1].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8233.328052633833,0,2062.741580692975,0,8248.065503178703,1544.6237932200313,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{-0.3688936756980066,0.4945288505113377,0.786993438372868,-0.4705592907909645,0.6308190119933399,-0.6169613666657347,-0.8015556187067238,-0.5979202205308677,1.1378777919901495e-9},
                    Vec3f{28.987623602443575,1.2869741681953482,-35.41913479007127},
                    Vec3f{31.357247031268316,-0.7630235707077014,-35.382114904849398},
                    Vec3f{0.653409298041993,0.7540864973647855,-0.0664066542176256},
                    Vec3f{0.8074732002435436,-0.5898978715137748,0.0027444620544127414},
                     -5.349743366241455,
                     false, 
                     Vec2f{38.77520896474247,46.875686776908789}
                 }, // mirror sys param
                MirrorActuatorMapping({  361.5,           241.15900701957345,           42.847175257850249, 3.4898574883995945, 0.006937503931091383,1.000090951429092,-0.005203127948319609, true})
                ) );

//B2
    _cam_params[ModuleName::B2].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8255.864827236077,0,2087.7933649082383,0,8275.915353839093,1634.7781053512658,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{0.3519829681230551,0.470711642611313,-0.8090355614319087,-0.484495177770416,-0.6479220341820895,-0.5877596960821285,-0.800857298692156,0.5988552305286333,2.618332217707575e-10},
                    Vec3f{-20.414422130822229,-29.675654742051479,-44.8759373695172},
                    Vec3f{-26.395636362517366,-34.039450167059069,-43.409020961403957},
                    Vec3f{0.5859198142165962,-0.8101004895671928,-0.020860683387272526},
                    Vec3f{-0.8026178139285465,-0.5964936083058808,-0.00014147364300031237},
                     -11.046143531799317, 
                     true, 
                     Vec2f{41.913424441962167,46.21338640816947}
                 }, // mirror sys param
                MirrorActuatorMapping({247,
                                        142.0164309742597,
                                        44.08546011731278,
                                         1.843485861369375,
                                        0.013910244270054343,1.0003551653357648,-0.010432683202544749, true})
                ) );

    //B3
    _cam_params[ModuleName::B3].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8227.866292893945,0,2084.8329215911294,0,8263.551005728017,1603.304646223205,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{0.36335896286791016,-0.49586982263260728,-0.7887226274843795,0.46618917739100548,-0.6362004668257961,0.6147492309016226,-0.8066212958923502,-0.5910685958608742,2.9042176441507196e-9},
                        Vec3f{-0.4510901323566069,40.968872924158507,-37.83220251280587},
                        Vec3f{-6.05891086158244,45.02289518248879,-42.54633066221509},
                        Vec3f{-0.5885503092606687,-0.8084458379917265,0.004884721386753081},
                        Vec3f{-0.7939891385281993,0.6079310186605894,-0.0010604006536922015},
                     -12.821348190307618, 
                     true, 
                     Vec2f{35.87328227854804,46.7039845304485}
                 }, // mirror sys param
                MirrorActuatorMapping({511.75,
                                        287.76538476103527,
                                         41.27573733120064,
                                         4.664458190585415,
                                       -0.000270684163047764,1.0000014482089433,0.00020301312228465117, true})
                ) );

    //B5
    _cam_params[ModuleName::B5].reset( new(Heap::DDR) CalibDataWMirror( 
                Matrix3x3f({8217.001114110119,0,2054.2803749933714,0,8240.529974811792,1593.1266770451493,0,0,1}), 
                MirrorSysParam
                    { Matrix3x3f{-0.3620438861429338,-0.469809664483511,0.8051106157941362,0.49143887919232295,0.6377232406645863,0.59312468869071,-0.7920934619883666,0.6103998259126136,-0.0000010653863742327019},
                    Vec3f{-37.39919579487915,27.383233657023433,-17.678970593780507},
                    Vec3f{-33.08957864344309,31.89067236278645,-15.517347468015109},
                    Vec3f{-0.7533559356303505,0.6302220078527002,0.1878165463121676},
                    Vec3f{0.8411200762912576,0.538886352460283,-0.04602734395813901},   
                     5.301071643829346,
                     false, 
                     Vec2f{34.54689139359751,45.0624898287135}
                 }, // mirror sys param
                MirrorActuatorMapping({437.25,
                                        296.2716940017501,
                                        39.8335124041213,
                                        4.656318775903197,
                                        0.13760928411207508,0.9972452251306706,-0.10320696308405718, true})
                ) );
}

#endif

} // end namespace ltaf
