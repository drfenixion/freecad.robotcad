set (sdfs
  actor.sdf
  air_pressure.sdf
  altimeter.sdf
  atmosphere.sdf
  audio_source.sdf
  audio_sink.sdf
  battery.sdf
  box_shape.sdf
  camera.sdf
  collision.sdf
  collision_engine.sdf
  contact.sdf
  cylinder_shape.sdf
  frame.sdf
  forcetorque.sdf
  geometry.sdf
  gps.sdf
  gripper.sdf
  gui.sdf
  heightmap_shape.sdf
  image_shape.sdf
  imu.sdf
  inertial.sdf
  joint.sdf
  lidar.sdf
  light.sdf
  light_state.sdf
  link.sdf
  link_state.sdf
  logical_camera.sdf
  magnetometer.sdf
  material.sdf
  mesh_shape.sdf
  model.sdf
  model_state.sdf
  navsat.sdf
  noise.sdf
  particle_emitter.sdf
  physics.sdf
  plane_shape.sdf
  plugin.sdf
  polyline_shape.sdf
  population.sdf
  pose.sdf
  projector.sdf
  ray.sdf
  rfidtag.sdf
  rfid.sdf
  road.sdf
  root.sdf
  scene.sdf
  sensor.sdf
  spherical_coordinates.sdf
  sphere_shape.sdf
  sonar.sdf
  state.sdf
  surface.sdf
  transceiver.sdf
  urdf.sdf
  visual.sdf
  world.sdf
)

set (SDF_SCHEMA)

foreach(FIL ${sdfs})
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)

  list(APPEND SDF_SCHEMA "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.xsd")

  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.xsd"
    COMMAND ${RUBY} ${CMAKE_SOURCE_DIR}/tools/xmlschema.rb
    ARGS -s ${CMAKE_CURRENT_SOURCE_DIR} -i ${ABS_FIL} -o ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${ABS_FIL}
    COMMENT "Running xml schema compiler on ${FIL}"
    VERBATIM)
endforeach()

add_custom_target(schema1_7 ALL DEPENDS ${SDF_SCHEMA})

set_source_files_properties(${SDF_SCHEMA} PROPERTIES GENERATED TRUE)

install(FILES 1_6.convert ${sdfs} DESTINATION ${CMAKE_INSTALL_FULL_DATAROOTDIR}/sdformat${PROJECT_VERSION_MAJOR}/1.7)
install(FILES ${SDF_SCHEMA} DESTINATION ${CMAKE_INSTALL_FULL_DATAROOTDIR}/sdformat${PROJECT_VERSION_MAJOR}/1.7)
