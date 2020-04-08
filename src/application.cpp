#include "application.h"
#include "dynamic_scene/ambient_light.h"
#include "dynamic_scene/environment_light.h"
#include "dynamic_scene/directional_light.h"
#include "dynamic_scene/area_light.h"
#include "dynamic_scene/point_light.h"
#include "dynamic_scene/spot_light.h"
#include "dynamic_scene/sphere.h"
#include "dynamic_scene/mesh.h"
#include "dynamic_scene/widgets.h"

#include "CMU462/lodepng.h"

#include "GLFW/glfw3.h"

#include <functional>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <thread>
using namespace std;

using namespace DynamicScene;

using Collada::CameraInfo;
using Collada::LightInfo;
using Collada::MaterialInfo;
using Collada::PolymeshInfo;
using Collada::SceneInfo;
using Collada::SphereInfo;

namespace CMU462 {

Application::Application(AppConfig config) {
  scene = nullptr;

  timestep = 0.1;
  damping_factor = 0.0;

  useCapsuleRadius = true;
}

Application::~Application() {
  if (scene != nullptr) delete scene;
}

void Application::init() {
  if (scene != nullptr) {
    delete scene;
    scene = nullptr;
  }

  textManager.init(use_hdpi);
  text_color = Color(1.0, 1.0, 1.0);

  // Setup all the basic internal state to default values,
  // as well as some basic OpenGL state (like depth testing
  // and lighting).

  // Set the integer bit vector representing which keys are down.
  leftDown = false;
  rightDown = false;
  middleDown = false;

  show_coordinates = true;
  show_hud = true;

  // Lighting needs to be explicitly enabled.
  glEnable(GL_LIGHTING);

  // Enable anti-aliasing and circular points.
  glEnable(GL_LINE_SMOOTH);
  // glEnable( GL_POLYGON_SMOOTH ); // XXX causes cracks!
  glEnable(GL_POINT_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  // glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

  // initialize fbos
  glGenFramebuffers(1, &backface_fbo);
  glGenFramebuffers(1, &frntface_fbo);
  glGenTextures(1, &backface_color_tex);
  glGenTextures(1, &backface_depth_tex);
  glGenTextures(1, &frntface_color_tex);
  glGenTextures(1, &frntface_depth_tex);

  // Initialize styles (colors, line widths, etc.) that will be used
  // to draw different types of mesh elements in various situations.
  initialize_style();

  mode = MODEL_MODE;
  action = Action::Navigate;
  scene = nullptr;

  // Make a dummy camera so resize() doesn't crash before the scene has been
  // loaded.
  // NOTE: there's a chicken-and-egg problem here, because load()
  // requires init, and init requires init_camera (which is only called by
  // load()).
  screenW = screenH = 600;  // Default value
  CameraInfo cameraInfo;
  cameraInfo.hFov = 20;
  cameraInfo.vFov = 28;
  cameraInfo.nClip = 0.1;
  cameraInfo.fClip = 100;
  camera.configure(cameraInfo, screenW, screenH);
  canonicalCamera.configure(cameraInfo, screenW, screenH);

  // Now initialize the timeline
  timeline.setMaxFrame(300);
  timeline.action_rewind();
  timeline.resize(screenW, 64);
  timeline.move(0, screenH - 64);
  timeline.markTime(0);
}

void Application::initialize_style() {
  // Colors.

  auto COLOR_SELECT = Color(0.0, 1.0, 0.0, 1.0);
  auto COLOR_HOVER = Color(0.8, 1.0, 0.8, 1.0);
  auto COLOR_DEFAULT = Color(0.0, 0.0, 0.0, 1.0);

  defaultStyle.halfedgeColor = COLOR_DEFAULT;
  hoverStyle.halfedgeColor = COLOR_HOVER;
  selectStyle.halfedgeColor = COLOR_SELECT;

  defaultStyle.faceColor = COLOR_DEFAULT;
  hoverStyle.faceColor = COLOR_HOVER;
  selectStyle.faceColor = COLOR_SELECT;

  defaultStyle.edgeColor = COLOR_DEFAULT;
  hoverStyle.edgeColor = COLOR_HOVER;
  selectStyle.edgeColor = COLOR_SELECT;

  defaultStyle.vertexColor = COLOR_DEFAULT;
  hoverStyle.vertexColor = COLOR_HOVER;
  selectStyle.vertexColor = COLOR_SELECT;

  defaultStyle.jointColor = COLOR_DEFAULT;
  hoverStyle.jointColor = COLOR_HOVER;
  selectStyle.jointColor = COLOR_SELECT;

  // Primitive sizes.
  defaultStyle.strokeWidth = 0.5;
  hoverStyle.strokeWidth = 2.0;
  selectStyle.strokeWidth = 2.0;

  defaultStyle.vertexRadius = 4.0;
  hoverStyle.vertexRadius = 8.0;
  selectStyle.vertexRadius = 8.0;
}

void Application::enter_2D_GL_draw_mode() {
  int screen_w = screenW;
  int screen_h = screenH;
  glPushAttrib(GL_VIEWPORT_BIT);
  glViewport(0, 0, screen_w, screen_h);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, screen_w, screen_h, 0, 0, 1);  // Y flipped !
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(0, 0, -1);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
}

void Application::exit_2D_GL_draw_mode() {
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glPopAttrib();
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

void Application::update_style() {
  float view_distance = (camera.position() - camera.view_point()).norm();
  float scale_factor = canonical_view_distance / view_distance;

  hoverStyle.strokeWidth = 2.0 * scale_factor;
  selectStyle.strokeWidth = 2.0 * scale_factor;

  hoverStyle.vertexRadius = 8.0 * scale_factor;
  selectStyle.vertexRadius = 8.0 * scale_factor;
}

void Application::render() {
  // Update the hovered element using the pick buffer once very n iterations.
  // We do this here rather than on mouse move, because some platforms generate
  // an excessive number of mouse move events which incurs a performance hit.
  if(pickDrawCountdown < 0) {
    Vector2D p(mouseX, screenH - mouseY);
    scene->getHoveredObject(p);
    pickDrawCountdown += pickDrawInterval;
  } else {
    pickDrawCountdown--;
  }

  glClearColor(0.18, 0.36, 0.69, 0.);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  update_gl_camera();

  // Need to clear depth buffers after calling getMouseProjection
  glBindFramebuffer(GL_FRAMEBUFFER, backface_fbo);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glBindFramebuffer(GL_FRAMEBUFFER, frntface_fbo);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // Rebind framebuffer to active
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  std::function<void(bool)> render_fn = [](bool b){};

  switch (mode) {
    case MODEL_MODE:
      render_fn = [this](bool depth){
        if (!depth && show_coordinates) draw_coordinates();

        scene->render_in_opengl();

        if (!depth && show_hud) draw_hud();
      };
  }

  { // scope for using statements
    using CMU462::DynamicScene::Mesh;
    using CMU462::DynamicScene::RenderMask;

    //glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    Mesh::flip_normals = true;
    Mesh::global_render_mask = RenderMask::FACE;

    glBindFramebuffer(GL_FRAMEBUFFER, backface_fbo);
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);
    render_fn(true);

    Mesh::flip_normals = false;

    glBindFramebuffer(GL_FRAMEBUFFER, frntface_fbo);
    glFrontFace(GL_CCW);
    render_fn(true);

    glDisable(GL_CULL_FACE);
    Mesh::global_render_mask = RenderMask::ALL;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    render_fn(false);
  }
}

void Application::update_gl_camera() {
  // Call resize() every time we draw, since it doesn't seem
  // to get called by the Viewer upon initial window creation
  // (this should probably be fixed!).
  GLint view[4];
  glGetIntegerv(GL_VIEWPORT, view);
  if (view[2] != screenW || view[3] != screenH) {
    resize(view[2], view[3]);
  }

  // Control the camera to look at the mesh.
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const Vector3D &c = camera.position();
  const Vector3D &r = camera.view_point();
  const Vector3D &u = camera.up_dir();

  gluLookAt(c.x, c.y, c.z, r.x, r.y, r.z, u.x, u.y, u.z);
}

void Application::resize(size_t w, size_t h) {
  screenW = w;
  screenH = h;
  camera.set_screen_size(w, h);
  textManager.resize(w, h);
  set_projection_matrix();
  timeline.resize(w, 64);
  timeline.move(0, h - 64);

  auto set_params = [](bool depth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    if(depth) {
      glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    }
  };

  // update backface fbo texture
  glBindFramebuffer(GL_FRAMEBUFFER, backface_fbo);
  glBindTexture(GL_TEXTURE_2D, backface_color_tex);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA8, screenW, screenH, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
  set_params(false);
  glBindTexture(GL_TEXTURE_2D, backface_depth_tex);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, screenW, screenH, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
  set_params(true);

  glFramebufferTexture2D(
    GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, backface_color_tex, 0
  );
  glFramebufferTexture2D(
    GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, backface_depth_tex, 0
  );


  // update frontface fbo texture
  glBindFramebuffer(GL_FRAMEBUFFER, frntface_fbo);
  glBindTexture(GL_TEXTURE_2D, frntface_color_tex);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA8, screenW, screenH, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
  set_params(false);
  glBindTexture(GL_TEXTURE_2D, frntface_depth_tex);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, screenW, screenH, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
  set_params(true);

  glFramebufferTexture2D(
    GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, frntface_color_tex, 0
  );
  glFramebufferTexture2D(
    GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, frntface_depth_tex, 0
  );
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Application::set_projection_matrix() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(camera.v_fov(), camera.aspect_ratio(), camera.near_clip(),
                 camera.far_clip());
}

string Application::name() { return "Loop Subdivision"; }

string Application::info() {
  switch (mode) {
    case MODEL_MODE:
      return "MeshEdit";
      break;
  }
}

void Application::load(SceneInfo *sceneInfo) {
  vector<Collada::Node> &nodes = sceneInfo->nodes;
  vector<DynamicScene::SceneLight *> lights;
  vector<DynamicScene::SceneObject *> objects;

  // save camera position to update camera control later
  CameraInfo *c;
  Vector3D c_pos = Vector3D();
  Vector3D c_dir = Vector3D();

  int len = nodes.size();
  for (int i = 0; i < len; i++) {
    Collada::Node &node = nodes[i];
    Collada::Instance *instance = node.instance;
    const Matrix4x4 &transform = node.transform;

    switch (instance->type) {
      case Collada::Instance::CAMERA:
        c = static_cast<CameraInfo *>(instance);
        c_pos = (transform * Vector4D(c_pos, 1)).to3D();
        c_dir = (transform * Vector4D(c->view_dir, 1)).to3D().unit();
        init_camera(*c, transform);
        break;
      case Collada::Instance::LIGHT: {
        lights.push_back(
            init_light(static_cast<LightInfo &>(*instance), transform));
        break;
      }
      case Collada::Instance::SPHERE:
        objects.push_back(
            init_sphere(static_cast<SphereInfo &>(*instance), transform));
        break;
      case Collada::Instance::POLYMESH:
        objects.push_back(
            init_polymesh(static_cast<PolymeshInfo &>(*instance), transform));
        break;
      case Collada::Instance::MATERIAL:
        init_material(static_cast<MaterialInfo &>(*instance));
        break;
    }
  }

  if (lights.size() == 0) {  // no lights, default use ambient_light
    LightInfo default_light = LightInfo();
    lights.push_back(new DynamicScene::AmbientLight(default_light));
  }
  scene = new DynamicScene::Scene(objects, lights);

  const BBox &bbox = scene->get_bbox();
  if (!bbox.empty()) {
    Vector3D target = bbox.centroid();
    canonical_view_distance = bbox.extent.norm() / 2 * 1.5;

    double view_distance = canonical_view_distance * 2;
    double min_view_distance = canonical_view_distance / 10.0;
    double max_view_distance = canonical_view_distance * 20.0;

    canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                          view_distance, min_view_distance, max_view_distance);

    camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
                 min_view_distance, max_view_distance);

    set_scroll_rate();
  }

  // set default draw styles for meshEdit -
  scene->set_draw_styles(&defaultStyle, &hoverStyle, &selectStyle);
}

void Application::init_camera(CameraInfo &cameraInfo,
                              const Matrix4x4 &transform) {
  camera.configure(cameraInfo, screenW, screenH);
  canonicalCamera.configure(cameraInfo, screenW, screenH);
  set_projection_matrix();
}

void Application::reset_camera() { camera.copy_placement(canonicalCamera); }

DynamicScene::SceneLight *Application::init_light(LightInfo &light,
                                                  const Matrix4x4 &transform) {
  switch (light.light_type) {
    case Collada::LightType::NONE:
      break;
    case Collada::LightType::AMBIENT:
      return new DynamicScene::AmbientLight(light);
    case Collada::LightType::DIRECTIONAL:
      return new DynamicScene::DirectionalLight(light, transform);
    case Collada::LightType::AREA:
      return new DynamicScene::AreaLight(light, transform);
    case Collada::LightType::POINT:
      return new DynamicScene::PointLight(light, transform);
    case Collada::LightType::SPOT:
      return new DynamicScene::SpotLight(light, transform);
    default:
      break;
  }
  return nullptr;
}

/**
 * The transform is assumed to be composed of translation, rotation, and
 * scaling, where the scaling is uniform across the three dimensions; these
 * assumptions are necessary to ensure the sphere is still spherical. Rotation
 * is ignored since it's a sphere, translation is determined by transforming the
 * origin, and scaling is determined by transforming an arbitrary unit vector.
 */
DynamicScene::SceneObject *Application::init_sphere(
    SphereInfo &sphere, const Matrix4x4 &transform) {
  const Vector3D &position = (transform * Vector4D(0, 0, 0, 1)).projectTo3D();
  double scale = (transform * Vector4D(1, 0, 0, 0)).to3D().norm();
  return new DynamicScene::Sphere(sphere, position, scale);
}

DynamicScene::SceneObject *Application::init_polymesh(
    PolymeshInfo &polymesh, const Matrix4x4 &transform) {
  return new DynamicScene::Mesh(polymesh, transform);
}

void Application::set_scroll_rate() {
  scroll_rate = canonical_view_distance / 10;
}

void Application::init_material(MaterialInfo &material) {
  // TODO : Support Materials.
}

void Application::cursor_event(float x, float y) {
  if (leftDown) {
    mouse1_dragged(x, y);
  } else if (middleDown || rightDown) {
    mouse2_dragged(x, y);
  }

  mouseX = x;
  mouseY = y;
}

void Application::scroll_event(float offset_x, float offset_y) {
  // update_style();

  switch (mode) {
    case MODEL_MODE:
      switch (action) {
        case Action::Navigate:
          camera.move_forward(offset_y * scroll_rate);
          break;
        default:
          break;
      }
  }
}

void Application::mouse_event(int key, int event, unsigned char mods) {
  switch (event) {
    case EVENT_PRESS:
      switch (key) {
        case MOUSE_LEFT:
          mouse_pressed(LEFT);
          break;
        case MOUSE_RIGHT:
          mouse_pressed(RIGHT);
          break;
        case MOUSE_MIDDLE:
          mouse_pressed(MIDDLE);
          break;
      }
      break;
    case EVENT_RELEASE:
      switch (key) {
        case MOUSE_LEFT:
          mouse_released(LEFT);
          break;
        case MOUSE_RIGHT:
          mouse_released(RIGHT);
          break;
        case MOUSE_MIDDLE:
          mouse_released(MIDDLE);
          break;
      }
      break;
  }
}

void Application::char_event(unsigned int codepoint) {
  bool queued = false;

  switch (mode) {
    case MODEL_MODE:
      switch (codepoint) {
        case 'u':
          scene->upsample_selected_face();
          break;
        case 'U':
          scene->upsample_all_mesh(PI / 16);
          break;
        //case 'd':
        //case 'D':
        //  scene->downsample_selected_mesh();
        //  break;
        //case 'i':
        //case 'I':
        //  // i for isotropic.
        //  scene->resample_selected_mesh();
        //  break;
        case 'r':
        case 'R':
          reload();
          break;
        case 'f':
        case 'F':
          scene->flip_selected_edge();
          break;
        case 'p':
          scene->split_selected_edge();
          break;
        case 'c':
        case 'C':
          scene->collapse_selected_element();
          break;
        case 'n':
        case 'N':
          scene->selectNextHalfedge();
          break;
        case 't':
          scene->selectTwinHalfedge();
          break;
        case 'T':
          scene->triangulateSelection();
          break;
        //case 's':
        //  // Catmull-Clark subdivision
        //  scene->subdivideSelection(true);
        //  break;
        //case 'S':
        //  // linear subdivision
        //  scene->subdivideSelection(false);
        //  break;
        case 'h':
          scene->selectHalfedge();
          break;
        default:
          break;
      }
      break;
  }

  updateWidgets();
}

void Application::reload()
{
  load(defaultSceneInfo);
}

void Application::setDefaultSceneInfo(SceneInfo* sceneInfo)
{
  defaultSceneInfo = sceneInfo;
}

void Application::setGhosted(bool isGhosted) {
  scene->selected.clear();
  scene->removeObject(scene->elementTransform);

  this->isGhosted = isGhosted;
  for (auto object : scene->objects) {
    object->isGhosted = isGhosted;
  }
}

void Application::toggleGhosted() {
  scene->selected.clear();
  scene->removeObject(scene->elementTransform);

  isGhosted = !isGhosted;
  for (auto object : scene->objects) {
    object->isGhosted = isGhosted;
  }
}

void Application::updateWidgets() {
  if (scene->selected.object == nullptr) {
    scene->removeObject(scene->elementTransform);
    return;
  }
}

void Application::keyboard_event(int key, int event, unsigned char mods) {
  switch (mode) {
    case MODEL_MODE:
      switch (key) {
        case GLFW_KEY_BACKSPACE:
        case GLFW_KEY_DELETE:
          if (event == GLFW_PRESS) {
            scene->erase_selected_element();
          }
          break;
        case GLFW_KEY_TAB:
          if (event == GLFW_PRESS) {
            show_hud = !show_hud;
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

  if (lastEventWasModKey && event == GLFW_RELEASE) {
    scene->elementTransform->restoreLastMode();
  }

  lastEventWasModKey = false;
  if (mods) {
    lastEventWasModKey = true;

    switch (mods) {
      case GLFW_MOD_SHIFT:
        scene->elementTransform->setScale();
        break;
      case GLFW_MOD_CONTROL:
        scene->elementTransform->setTranslate();
        break;
      case GLFW_MOD_ALT:
        scene->elementTransform->setRotate();
        break;
      default:
        break;
    }
  }

  updateWidgets();
}

void Application::writeScene(const char *filename) {
  cerr << "Writing scene to file " << filename << endl;
  Collada::ColladaWriter::writeScene(*scene, filename);
}

void Application::loadScene(const char *filename) {
  cerr << "Loading scene from file " << filename << endl;

  Camera originalCamera = camera;
  Camera originalCanonicalCamera = canonicalCamera;

  Collada::SceneInfo *sceneInfo = new Collada::SceneInfo();
  if (Collada::ColladaParser::load(filename, sceneInfo) < 0) {
    cerr << "Warning: scene file failed to load." << endl;
    delete sceneInfo;
    return;
  }
  load(sceneInfo);

  camera = originalCamera;
  canonicalCamera = originalCanonicalCamera;
}

Vector3D Application::getMouseProjection(double dist) {
  // get projection matrix from OpenGL stack.
  GLdouble projection[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  Matrix4x4 projection_matrix(projection);

  // get view matrix from OpenGL stack.
  GLdouble modelview[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  Matrix4x4 modelview_matrix(modelview);

  // ray in clip coordinates
  double x = mouseX * 2 / screenW - 1;
  double y = screenH - mouseY;  // y is upside down
  y = y * 2 / screenH - 1;

  Vector4D ray_clip(x, y, -1.0, 1.0);
  // ray in eye coordinates
  Vector4D ray_eye = projection_matrix.inv() * ray_clip;

  // ray is into the screen and not a point.
  ray_eye.z = -1.0;
  ray_eye.w = 0.0;

  GLfloat z1, z2;
  const float zNear = camera.near_clip();
  const float zFar = camera.far_clip();
  glBindFramebuffer(GL_READ_FRAMEBUFFER, frntface_fbo);
  glReadPixels(mouseX, screenH-mouseY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z1);
  z1 = z1 * 2.f - 1.f;
  z1 = 2.f * zNear * zFar / (zFar + zNear - z1 * (zFar - zNear));
  glBindFramebuffer(GL_READ_FRAMEBUFFER, backface_fbo);
  glReadPixels(mouseX, screenH-mouseY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z2);
  z2 = z2 * 2.f - 1.f;
  z2 = 2.f * zNear * zFar / (zFar + zNear - z2 * (zFar - zNear));
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

  bool invalid = (z1 <= zNear || z2 <= zNear || z1 >= zFar || z2 >= zFar);

  // ray in world coordinates
  Vector4D ray_wor4 = modelview_matrix * ray_eye;
  Vector3D ray_wor(ray_wor4.x, ray_wor4.y, ray_wor4.z);

  Vector3D ray_orig(camera.position());

  double t = dot(ray_orig, -ray_wor);
  if(std::isfinite(dist)) {
    // If a distance was given, use that instead
    ray_wor = ray_wor.unit();
    t = dist;
  } else if(!invalid) {
    // The current ray  is not a unit vector - it is normalized in z
    // so we can simply move along that (scaled) ray by the viewspace
    // z value
    t = z1 + (z2 - z1) / 2.f; // average of in/out depth
  }

  Vector3D intersect = ray_orig + t * ray_wor;

  return intersect;
}

void Application::mouse_pressed(e_mouse_button b) {

  Vector2D p(mouseX, screenH - mouseY);
  scene->getHoveredObject(p);

  switch (b) {
    case LEFT:
      leftDown = true;
      if (mode == MODEL_MODE) {
        selectHovered();
      }
      break;
    case RIGHT:
      rightDown = true;
      break;
    case MIDDLE:
      middleDown = true;
      break;
  }
  updateWidgets();
}

void Application::selectHovered() {
  scene->selected = scene->hovered;

  scene->elementTransform->setClickPosition(Vector2D(mouseX, mouseY));

  scene->edited.clear();
}

void Application::mouse_released(e_mouse_button b) {
  switch (b) {
    case LEFT:
      leftDown = false;
      draggingTimeline = false;
      scene->elementTransform->updateGeometry();
      scene->elementTransform->onMouseReleased();
      break;
    case RIGHT:
      rightDown = false;
      break;
    case MIDDLE:
      middleDown = false;
      break;
  }

  updateWidgets();
}

/*
  When in edit mode and there is a selection, move the selection.
  When in visualization mode, rotate.
*/
void Application::mouse1_dragged(float x, float y) {
  float dx = -(x - mouseX);
  float dy = -(y - mouseY);
  if (mode == MODEL_MODE) {
    switch (action) {
      case (Action::Navigate):
        camera.rotate_by(dy * (PI / screenH), dx * (PI / screenW));
        break;
      default:
        break;
    }
  }

  updateWidgets();
}

/*
  When the mouse is dragged with the right button held down, translate.
*/
void Application::mouse2_dragged(float x, float y) {
  float dx = (x - mouseX);
  float dy = (y - mouseY);
  // don't negate y because up is down.
  camera.move_by(-dx, dy, canonical_view_distance);

  updateWidgets();
}

Matrix4x4 Application::get_world_to_3DH() {
  Matrix4x4 P, V, M;
  glGetDoublev(GL_PROJECTION_MATRIX, &P(0, 0));
  auto sel = scene->selected.object;
  if(sel) {
    if(sel == scene->elementTransform)
      M = scene->elementTransform->target.object->getTransformation();
    else
      M = sel->getTransformation();
  }
  else
    M = Matrix4x4::identity();
  V = camera.getTransformation();
  return P * V * M;
}

inline void Application::draw_string(float x, float y, string str, size_t size,
                                     const Color &c) {
  int line_index = textManager.add_line((x * 2 / screenW) - 1.0,
                                        (-y * 2 / screenH) + 1.0, str, size, c);
  textManager.set_size(line_index, size);
  messages.push_back(line_index);
}

void Application::draw_coordinates() {
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  glLineWidth(1.);

  glBegin(GL_LINES);
  glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
  glVertex3i(0, 0, 0);
  glVertex3i(1, 0, 0);

  glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
  glVertex3i(0, 0, 0);
  glVertex3i(0, 1, 0);

  glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
  glVertex3i(0, 0, 0);
  glVertex3i(0, 0, 1);

  glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
  for (int x = 0; x <= 10; ++x) {
    glVertex3i(x - 5, 0, -5);
    glVertex3i(x - 5, 0, 5);
  }
  for (int z = 0; z <= 10; ++z) {
    glVertex3i(-5, 0, z - 5);
    glVertex3i(5, 0, z - 5);
  }
  glEnd();

  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
}

void Application::draw_hud() {
  textManager.clear();
  messages.clear();

  const size_t size = 16;
  const float x0 = use_hdpi ? screenW - 180 * 2 : screenW - 100;
  const float y0 = use_hdpi ? 32 : 16;
  const int inc = use_hdpi ? 24 : 12;
  float y = y0 + inc - size;

  // No selection --> no messages.
  if (!scene->has_selection()) {
	int nVertices = 0;
    int nEdges = 0;
    int nHalfedges = 0;
    int nFaces = 0;
    int objCount = 0;
    for (auto obj : scene->objects) {
      auto m = dynamic_cast<Mesh*>(obj);
      if (m) {
        objCount++;
        nVertices += m->mesh.nVertices();
        nEdges += m->mesh.nEdges();
        nHalfedges += m->mesh.nHalfedges();
        nFaces += m->mesh.nFaces();
      }
    }
    draw_string(x0, y, string("SCENE"), size, text_color);
    y += inc;
    draw_string(x0, y, string("Vertex Count: ") + to_string(nVertices), size, text_color);
    y += inc;
    draw_string(x0, y, string("Edge Count: ") + to_string(nEdges), size, text_color);
    y += inc;
    draw_string(x0, y, string("Halfedge Count: ") + to_string(nHalfedges), size, text_color);
    y += inc;
    draw_string(x0, y, string("Face Count: ") + to_string(nFaces), size, text_color);
    y += inc;
    draw_string(x0, y, string("Object Count: ") + to_string(objCount), size, text_color);
    y += inc;
  } else {
    Info selectionInfo = scene->getSelectionInfo();
    for (const string &s : selectionInfo) {
      draw_string(x0, y, s, size, text_color);
      y += inc;
    }
  }

  // -- First draw a lovely black rectangle.

  glPushAttrib(GL_VIEWPORT_BIT);
  glViewport(0, 0, screenW, screenH);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, screenW, screenH, 0, 0, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(0, 0, -1);


  float min_x = x0 - 32;
  float min_y = y0 - 32;
  float max_x = screenW;
  float max_y = y;

  float z = 0.0;

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glColor4f(0.0, 0.0, 0.0, 0.6);

  glBegin(GL_QUADS);

  glVertex3f(min_x, min_y, z);
  glVertex3f(min_x, max_y, z);
  glVertex3f(max_x, max_y, z);
  glVertex3f(max_x, min_y, z);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glPopAttrib();

  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  textManager.render();
}

}