API reference
=============

.. contents:: :local:

MjSim: Basic simulation
-----------------------

.. autofunction:: mujoco_py.load_model_from_path(path)

.. autofunction:: mujoco_py.load_model_from_xml(xml_string)

.. autofunction:: mujoco_py.load_model_from_mjb(path)

.. autoclass:: mujoco_py.MjSim(model, data=None, nsubsteps=1, udd_callback=None)
    :members: model, data, step, render, get_state, set_state, set_state_from_flattened, save, reset

.. autoclass:: mujoco_py.MjSimState

.. autofunction:: mujoco_py.ignore_mujoco_warnings

.. _pymjdata:

PyMjData: Time-dependent data
-----------------------------

``PyMjData`` and related classes are automatically generated from the MuJoCo C header files. For more information on this process, see :ref:`genwrapper`. Their structure therefore directly follows the MuJoCo structs.

.. raw:: html

    <dl class="class">
    <dt id="mujoco_py.PyMjData">
    <em class="property">class </em><code class="descclassname">mujoco_py.</code><code class="descname">PyMjData</code><a class="headerlink" href="#mujoco_py.PyMjData" title="Permalink to this definition">¶</a></dt>
    <dd>
    <p class="rubric">Attributes</p>

.. attribute:: act
.. attribute:: act_dot
.. attribute:: active_contacts_efc_pos
.. attribute:: actuator_force
.. attribute:: actuator_length
.. attribute:: actuator_moment
.. attribute:: actuator_velocity
.. attribute:: body_jacp
.. attribute:: body_jacr
.. attribute:: body_xmat
.. attribute:: body_xpos
.. attribute:: body_xquat
.. attribute:: body_xvelp
.. attribute:: body_xvelr
.. attribute:: cacc
.. attribute:: cam_xmat
.. attribute:: cam_xpos
.. attribute:: cdof
.. attribute:: cdof_dot
.. attribute:: cfrc_ext
.. attribute:: cfrc_int
.. attribute:: cinert
.. attribute:: contact
.. attribute:: crb
.. attribute:: ctrl
.. attribute:: cvel
.. attribute:: efc_AR
.. attribute:: efc_AR_colind
.. attribute:: efc_AR_rowadr
.. attribute:: efc_AR_rownnz
.. attribute:: efc_D
.. attribute:: efc_J
.. attribute:: efc_JT
.. attribute:: efc_JT_colind
.. attribute:: efc_JT_rowadr
.. attribute:: efc_JT_rownnz
.. attribute:: efc_J_colind
.. attribute:: efc_J_rowadr
.. attribute:: efc_J_rownnz
.. attribute:: efc_R
.. attribute:: efc_aref
.. attribute:: efc_b
.. attribute:: efc_diagApprox
.. attribute:: efc_force
.. attribute:: efc_frictionloss
.. attribute:: efc_id
.. attribute:: efc_margin
.. attribute:: efc_solimp
.. attribute:: efc_solref
.. attribute:: efc_state
.. attribute:: efc_type
.. attribute:: efc_vel
.. attribute:: energy
.. attribute:: geom_jacp
.. attribute:: geom_jacr
.. attribute:: geom_xmat
.. attribute:: geom_xpos
.. attribute:: geom_xvelp
.. attribute:: geom_xvelr
.. attribute:: light_xdir
.. attribute:: light_xpos
.. attribute:: maxuse_con
.. attribute:: maxuse_efc
.. attribute:: maxuse_stack
.. attribute:: mocap_pos
.. attribute:: mocap_quat
.. attribute:: nbuffer
.. attribute:: ncon
.. attribute:: ne
.. attribute:: nefc
.. attribute:: nf
.. attribute:: nstack
.. attribute:: pstack
.. attribute:: qLD
.. attribute:: qLDiagInv
.. attribute:: qLDiagSqrtInv
.. attribute:: qM
.. attribute:: qacc
.. attribute:: qacc_unc
.. attribute:: qacc_warmstart
.. attribute:: qfrc_actuator
.. attribute:: qfrc_applied
.. attribute:: qfrc_bias
.. attribute:: qfrc_constraint
.. attribute:: qfrc_inverse
.. attribute:: qfrc_passive
.. attribute:: qfrc_unc
.. attribute:: qpos
.. attribute:: qvel
.. attribute:: sensordata
.. attribute:: set_joint_qpos
.. attribute:: set_joint_qvel
.. attribute:: set_mocap_pos
.. attribute:: set_mocap_quat
.. attribute:: site_jacp
.. attribute:: site_jacr
.. attribute:: site_xmat
.. attribute:: site_xpos
.. attribute:: site_xvelp
.. attribute:: site_xvelr
.. attribute:: solver
.. attribute:: solver_fwdinv
.. attribute:: solver_iter
.. attribute:: solver_nnz
.. attribute:: subtree_angmom
.. attribute:: subtree_com
.. attribute:: subtree_linvel
.. attribute:: ten_length
.. attribute:: ten_moment
.. attribute:: ten_velocity
.. attribute:: ten_wrapadr
.. attribute:: ten_wrapnum
.. attribute:: time
.. attribute:: timer
.. attribute:: userdata
.. attribute:: warning
.. attribute:: wrap_obj
.. attribute:: wrap_xpos
.. attribute:: xanchor
.. attribute:: xaxis
.. attribute:: xfrc_applied
.. attribute:: ximat
.. attribute:: xipos

.. raw:: html

    <p class="rubric">Methods</p>


.. method:: get_body_jacp(name)

  Get the entry in ``jacp`` corresponding to the body with the given `name`

.. method:: get_body_jacr(name)

  Get the entry in ``jacr`` corresponding to the body with the given `name`

.. method:: get_body_ximat(name)

  Get the entry in ``ximat`` corresponding to the body with the given `name`

.. method:: get_body_xipos(name)

  Get the entry in ``xipos`` corresponding to the body with the given `name`

.. method:: get_body_xmat(name)

  Get the entry in ``xmat`` corresponding to the body with the given `name`

.. method:: get_body_xpos(name)

  Get the entry in ``xpos`` corresponding to the body with the given `name`

.. method:: get_body_xquat(name)

  Get the entry in ``xquat`` corresponding to the body with the given `name`

.. method:: get_body_xvelp(name)

  Get the entry in ``xvelp`` corresponding to the body with the given `name`

.. method:: get_body_xvelr(name)

  Get the entry in ``xvelr`` corresponding to the body with the given `name`

.. method:: get_cam_xmat(name)

  Get the entry in ``xmat`` corresponding to the cam with the given `name`

.. method:: get_cam_xpos(name)

  Get the entry in ``xpos`` corresponding to the cam with the given `name`

.. method:: get_camera_xmat(name)

  Get the entry in ``xmat`` corresponding to the camera with the given `name`

.. method:: get_camera_xpos(name)

  Get the entry in ``xpos`` corresponding to the camera with the given `name`

.. method:: get_geom_jacp(name)

  Get the entry in ``jacp`` corresponding to the geom with the given `name`

.. method:: get_geom_jacr(name)

  Get the entry in ``jacr`` corresponding to the geom with the given `name`

.. method:: get_geom_xmat(name)

  Get the entry in ``xmat`` corresponding to the geom with the given `name`

.. method:: get_geom_xpos(name)

  Get the entry in ``xpos`` corresponding to the geom with the given `name`

.. method:: get_geom_xvelp(name)

  Get the entry in ``xvelp`` corresponding to the geom with the given `name`

.. method:: get_geom_xvelr(name)

  Get the entry in ``xvelr`` corresponding to the geom with the given `name`

.. method:: get_joint_qpos(name)

  Get the entry in ``qpos`` corresponding to the joint with the given `name`

.. method:: get_joint_qvel(name)

  Get the entry in ``qvel`` corresponding to the joint with the given `name`

.. method:: get_joint_xanchor(name)

  Get the entry in ``xanchor`` corresponding to the joint with the given `name`

.. method:: get_joint_xaxis(name)

  Get the entry in ``xaxis`` corresponding to the joint with the given `name`

.. method:: get_light_xdir(name)

  Get the entry in ``xdir`` corresponding to the light with the given `name`

.. method:: get_light_xpos(name)

  Get the entry in ``xpos`` corresponding to the light with the given `name`

.. method:: get_mocap_pos(name)

  Get the entry in ``pos`` corresponding to the mocap with the given `name`

.. method:: get_mocap_quat(name)

  Get the entry in ``quat`` corresponding to the mocap with the given `name`

.. method:: get_site_jacp(name)

  Get the entry in ``jacp`` corresponding to the site with the given `name`

.. method:: get_site_jacr(name)

  Get the entry in ``jacr`` corresponding to the site with the given `name`

.. method:: get_site_xmat(name)

  Get the entry in ``xmat`` corresponding to the site with the given `name`

.. method:: get_site_xpos(name)

  Get the entry in ``xpos`` corresponding to the site with the given `name`

.. method:: get_site_xvelp(name)

  Get the entry in ``xvelp`` corresponding to the site with the given `name`

.. method:: get_site_xvelr(name)

  Get the entry in ``xvelr`` corresponding to the site with the given `name`

.. raw:: html

    </dd></dl>

MjSimPool: Batched simulation
-----------------------------

.. autoclass:: mujoco_py.MjSimPool
    :members: reset, forward, step, sims, create_from_sim

MjViewer: 3D rendering
-----------------------------

.. autoclass:: mujoco_py.MjViewerBasic
  :members: render

.. autoclass:: mujoco_py.MjViewer
  :members: render
