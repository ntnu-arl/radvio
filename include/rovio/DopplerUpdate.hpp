#ifndef ROVIO_DOPPLERUPDATE_HPP_
#define ROVIO_DOPPLERUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/RadarTarget.hpp"

namespace rovio
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

  V3D MvR_;  // radar-frame velocity expressed in IMU frame
  V3D RvR_;  // radar-frame velocity expressed in radar frame

  // Parameter
  double chirp_duration_;
  double range_min_;

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
  DopplerUpdate()
  {
    chirp_duration_ = 0.0;
    range_min_ = 0.0;
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
    doubleRegister_.registerScalar("chirp_duration_s", chirp_duration_);
    doubleRegister_.registerScalar("range_min_", range_min_);
  };

  /** \brief Destructor
   */
  virtual ~DopplerUpdate(){};

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
    const Target& t = meas_.aux().targets_[state.aux().activeTarget_];

    const double prediction = -t.bearing.dot(RvR_);
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
    const Target& t = meas_.aux().targets_[state.aux().activeTarget_];
    const V3D mu = t.bearing;
    const V3D w_hat = meas_.aux().BwWB_ - state.gyb();

    // not -t.bearing bc of ROVIO internal representation
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_vel>()) = mu.transpose() * rotmat;
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_gyb>()) = -mu.transpose() * rotmat * gSM(state.MrMR());
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_rep>()) = -mu.transpose() * gSM(rotmat * w_hat);
    F.template block<1, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
                           mtState::template getId<mtState::_rea>()) = mu.transpose() * rotmat * gSM(MvR_);
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

  void commonPreProcess(mtFilterState& filterState, const mtMeas& meas)
  {
    filterState.state_.aux().activeTarget_ = 0;
  }

  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished)
  {
    if (isFinished)
    {
      commonPreProcess(filterState, meas);
      isFinished = false;
    }

    typename mtFilterState::mtState& state = filterState.state_;

    // -state.MvM() bc of ROVIO internal representation
    MvR_ = -state.MvM() + (meas.aux().BwWB_ - state.gyb()).cross(state.MrMR());
    RvR_ = state.qRM().rotate(MvR_);

    const int& ID = state.aux().activeTarget_;
    const TargetVector& targets = meas.aux().targets_;

    if (ID >= targets.size())
      isFinished = true;
  }

  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection,
                   bool& isFinished)
  {
    filterState.state_.aux().activeTarget_++;
  }

  bool validTarget(const Target& target)
  {
    const double range = target.xyz.norm();
    if (range < range_min_)
    {
      return false;
    }

    return true;
  }
};

}  // namespace rovio

#endif /* ROVIO_DopplerUPDATE_HPP_ */
