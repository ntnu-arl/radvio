/*
* Copyright (c) 2026, Autonomous Robots Lab, Norwegian University of Science and Technology
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#ifndef RADVIO_DOPPLERUPDATE_HPP_
#define RADVIO_DOPPLERUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "radvio/FilterStates.hpp"
#include "radvio/RadarTarget.hpp"

namespace radvio
{
/** \brief Class, defining the innovation.
 */
class DopplerInnovation : public LWF::State<LWF::ScalarElement>
{
public:
  typedef LWF::State<LWF::ScalarElement> Base;
  using Base::E_;
  static constexpr unsigned int _doppler = 0;
  DopplerInnovation()
  {
    static_assert(_doppler + 1 == E_, "Error with indices");
    this->template getName<_doppler>() = "doppler";
  };
  virtual ~DopplerInnovation(){};
};

/** \brief Class, dummy auxillary class for Zero Doppler update
 */
class DopplerUpdateMeasAuxiliary : public LWF::AuxiliaryBase<DopplerUpdateMeasAuxiliary>
{
public:
  DopplerUpdateMeasAuxiliary(){};
  virtual ~DopplerUpdateMeasAuxiliary(){};
  TargetVector targets_;
  V3D BwWB_{ 0, 0, 0 };  // mid chirp angular rate
};

/**  \brief Empty measurement
 */
class DopplerUpdateMeas : public LWF::State<DopplerUpdateMeasAuxiliary>
{
public:
  typedef LWF::State<DopplerUpdateMeasAuxiliary> Base;
  using Base::E_;
  // static constexpr unsigned int _doppler = 0;
  // static constexpr unsigned int _aux = _doppler+1;
  static constexpr unsigned int _aux = 0;
  DopplerUpdateMeas()
  {
    static_assert(_aux + 1 == E_, "Error with indices");
    // this->template getName<_doppler>() = "doppler";
    // this->template getName<_aux>() = "aux";
  };
  virtual ~DopplerUpdateMeas(){};

  //@{
  /** \brief Get the auxiliary state of the DopplerUpdateMeas.
   *
   *  \see DopplerUpdateMeasAuxiliary
   *  @return the the auxiliary state of the DopplerUpdateMeas.
   */
  inline DopplerUpdateMeasAuxiliary& aux()
  {
    return this->template get<_aux>();
  }
  inline const DopplerUpdateMeasAuxiliary& aux() const
  {
    return this->template get<_aux>();
  }
  //@}
};

/**  \brief Class holding the update noise.
 */
class DopplerUpdateNoise : public LWF::State<LWF::ScalarElement>
{
public:
  typedef LWF::State<LWF::ScalarElement> Base;
  using Base::E_;
  static constexpr unsigned int _doppler = 0;
  DopplerUpdateNoise()
  {
    static_assert(_doppler + 1 == E_, "Error with indices");
    this->template getName<_doppler>() = "doppler";
  };
  virtual ~DopplerUpdateNoise(){};
};

/** \brief Outlier Detection.
 * ODEntry<Start entry, dimension of detection>
 */
class DopplerOutlierDetection
  : public LWF::OutlierDetection<LWF::ODEntry<DopplerInnovation::template getId<DopplerInnovation::_doppler>(), 1>>
{
public:
  virtual ~DopplerOutlierDetection(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the zero Doppler update
 */
template <typename FILTERSTATE>
class DopplerUpdate : public LWF::Update<DopplerInnovation, FILTERSTATE, DopplerUpdateMeas, DopplerUpdateNoise,
                                         DopplerOutlierDetection, false>
{
public:
  typedef LWF::Update<DopplerInnovation, FILTERSTATE, DopplerUpdateMeas, DopplerUpdateNoise, DopplerOutlierDetection,
                      false>
      Base;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::meas_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;

  bool updateFinished_{false};
  TargetVector inliers_;

  // Parameter
  double timeOffset_;
  double chirp_duration_;
  double range_min_;
  double azimuth_min_;
  double azimuth_max_;
  double elevation_min_;
  double elevation_max_;

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
  DopplerUpdate()
  {
    timeOffset_ = 0.0;
    chirp_duration_ = 0.0;
    range_min_ = 0.0;
    azimuth_min_ = -90;
    azimuth_max_ = 90;
    elevation_min_ = -90;
    elevation_max_ = 90;
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
    doubleRegister_.registerScalar("timeOffset", timeOffset_);
    doubleRegister_.registerScalar("chirp_duration_s", chirp_duration_);
    doubleRegister_.registerScalar("range_min", range_min_);
    doubleRegister_.registerScalar("azimuth_min_deg", azimuth_min_);
    doubleRegister_.registerScalar("azimuth_max_deg", azimuth_max_);
    doubleRegister_.registerScalar("elevation_min_deg", elevation_min_);
    doubleRegister_.registerScalar("elevation_max_deg", elevation_max_);
  };

  /** \brief Destructor
   */
  virtual ~DopplerUpdate(){};

  /**
   * @brief Calculate the radar-frame velocity as a function of the state vector
   *
   * @param state
   */
  V3D calcRadarVelocity(const mtState& state) const
  {
    // -state.MvM() bc of RADVIO internal representation
    const V3D MvR = -state.MvM() + (meas_.aux().BwWB_ - state.gyb()).cross(state.MrMR());
    return state.qRM().rotate(MvR);
  }

  /** \brief Compute the inovvation term
   *
   *  @param mtInnovation - Class, holding innovation data.
   *  @param state        - Filter %State.
   *  @param meas         - Not Used.
   *  @param noise        - Additive discrete Gaussian noise.
   *  @param dt           - Not used.
   */
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const
  {
    const V3D RvR = calcRadarVelocity(state);
    const Target& t = meas_.aux().targets_[state.aux().activeTarget_];

    const double prediction = -t.bearing.dot(RvR);
    const double doppler_error = prediction - t.radial_speed;

    y.template get<mtInnovation::_doppler>() = doppler_error + noise.template get<mtNoise::_doppler>();
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  @param F     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   *  @param meas  - Not used.
   *  @param dt    - Not used.
   */
  void jacState(MXD& F, const mtState& state) const
  {
    F.setZero();

    const Eigen::Matrix3d rotmat = MPD(state.qRM()).matrix();
    const V3D RvR = calcRadarVelocity(state);
    const Target& t = meas_.aux().targets_[state.aux().activeTarget_];
    const V3D mu = t.bearing;
    const V3D w_hat = meas_.aux().BwWB_ - state.gyb();

    // not -t.bearing bc of RADVIO internal representation
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_vel>()) = mu.transpose() * rotmat;
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_gyb>()) = -mu.transpose() * rotmat * gSM(state.MrMR());
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_rep>()) = -mu.transpose() * rotmat * gSM(w_hat);
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_rea>()) = mu.transpose() * gSM(RvR);
  }

  /** \brief Computes the Jacobian for the update step of the filter w.r.t. to the noise variables
   *
   *  @param G     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   *  @param meas  - Not used.
   *  @param dt    - Not used.
   */
  void jacNoise(MXD& G, const mtState& state) const
  {
    G.setZero();
    G = Eigen::Matrix<double, 1, 1>::Identity();
  }

  /** \brief Prepares the filter state for the update.
   *
   *  Summary:
   *  1. Set active target idx to 0
   *  2. clear and reserve inliers vector
   *
   *  @param filterState - Filter state.
   *  @param meas        - Update measurement.
   */
  void commonPreProcess(mtFilterState& filterState, const mtMeas& meas)
  {
    updateFinished_ = false;

    filterState.state_.aux().activeTarget_ = 0;

    inliers_.clear();
    inliers_.reserve(meas.aux().targets_.size());
  }

  /** \brief Pre-Processing for the doppler update.
   *
   *  Summary:
   *  1. Check if we've processed all targets
   *
   *  @param filterState - Filter state.
   *  @param meas        - Update measurement.
   *  @param isFinished  - True, if process has finished.
   */
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished)
  {
    if (isFinished)
    {
      commonPreProcess(filterState, meas);
      isFinished = false;
    }

    const int& ID = filterState.state_.aux().activeTarget_;
    const TargetVector& targets = meas.aux().targets_;

    if (ID >= targets.size())
      isFinished = true;
  }

  /** \brief Post-Processing for the doppler update.
   *
   *  Summary:
   *  1. Get inliers and fill vector
   *  2. Increment ID
   *
   *  @param filterState      - Filter state.
   *  @param meas             - Update measurement.
   *  @param outlierDetection - Outlier detection.
   *  @param isFinished       - True, if process has finished.
   */
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection,
                   bool& isFinished)
  {
    if (isFinished)
    {
      commonPostProcess(filterState, meas);
    }
    else
    {
      int& ID = filterState.state_.aux().activeTarget_;
      const TargetVector& targets = meas.aux().targets_;

      if (!outlierDetection.isOutlier(0))
      {
        inliers_.emplace_back(targets[ID]);
      }

      ID++;
    }
  }

  void commonPostProcess(mtFilterState& filterState, const mtMeas& meas)
  {
    updateFinished_ = true;
  }

  bool validTarget(const Target& target)
  {
    constexpr double rad2deg = 180.0 / M_PI;

    if ((target.range < range_min_) || (target.azimuth < azimuth_min_) || (target.azimuth > azimuth_max_) ||
        (target.elevation < elevation_min_) || (target.elevation > elevation_max_))
    {
      return false;
    }

    return true;
  }
};

}  // namespace radvio

#endif /* RADVIO_DOPPLERUPDATE_HPP_ */
