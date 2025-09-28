#ifndef ROVIO_DOPPLERUPDATE_HPP_
#define ROVIO_DOPPLERUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/RadarTarget.hpp"

namespace rovio
{
#define MAX_CLOUD_SIZE 100

/** \brief Class, defining the innovation.
 */
class DopplerInnovation : public LWF::State<LWF::VectorElement<MAX_CLOUD_SIZE>>
{
public:
  typedef LWF::State<LWF::VectorElement<MAX_CLOUD_SIZE>> Base;
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
  V3D BwWB_;  // mid chirp angular rate
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
class DopplerUpdateNoise : public LWF::State<LWF::VectorElement<MAX_CLOUD_SIZE>>
{
public:
  typedef LWF::State<LWF::VectorElement<MAX_CLOUD_SIZE>> Base;
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
  : public LWF::OutlierDetection<
        LWF::ODEntry<DopplerInnovation::template getId<DopplerInnovation::_doppler>(), MAX_CLOUD_SIZE>>
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

  V3D RvR_;                                          // radar-frame velocity expressed in radar frame
  QPD qMR_{ 0.9990482216, 0.0, 0.0436193874, 0.0 };  // TODO: rotation from {R} to {M}
  V3D MrMR_{ 0.01, 0.0, 0.05 };                      // TODO: translation from {M} to {R} in {M}

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
  DopplerUpdate()
  {
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
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
    std::cout << "eval inno" << '\n';
    const size_t num_targets = std::min((size_t)MAX_CLOUD_SIZE, meas_.aux().targets_.size());

    Eigen::Matrix<double, MAX_CLOUD_SIZE, 1> doppler_error;
    doppler_error.setZero();

    std::cout << "RvR: " << RvR_.transpose() << '\n';

    std::cout << "num_targets: " << num_targets << '\n';
    for (size_t i = 0; i < num_targets; ++i)
    {
      const Target& t = meas_.aux().targets_[i];
      doppler_error(i) = -t.bearing.dot(RvR_) - t.radial_speed;
    }

    std::cout << doppler_error.transpose() << '\n';

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
    std::cout << "F" << '\n';
    F.setZero();
    // F.template block<3, 3>(mtInnovation::template getId<mtInnovation::_doppler>(),
    //                        mtState::template getId<mtState::_doppler>()) = MPD(qAM_).matrix();
    const size_t num_targets = std::min((size_t)MAX_CLOUD_SIZE, meas_.aux().targets_.size());

    Eigen::Matrix3d rotmat = MPD(qMR_).matrix();
    std::cout << "MrMR: " << MrMR_.transpose() << '\n';
    std::cout << "qMR: " << qMR_ << '\n';
    for (size_t i = 0; i < num_targets; ++i)
    {
      const Target& t = meas_.aux().targets_[i];
      F.template block<1, 3>(i, mtState::template getId<mtState::_vel>()) = -t.bearing.transpose() * rotmat.transpose();
      F.template block<1, 3>(i, mtState::template getId<mtState::_gyb>()) =
          -t.bearing.transpose() * (rotmat.transpose() * gSM(MrMR_));
    }
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
    std::cout << "G" << '\n';
    G.setZero();
    G.template block<MAX_CLOUD_SIZE, MAX_CLOUD_SIZE>(mtInnovation::template getId<mtInnovation::_doppler>(),
                                                     mtNoise::template getId<mtNoise::_doppler>()) =
        Eigen::Matrix<double, MAX_CLOUD_SIZE, MAX_CLOUD_SIZE>::Identity();
  }

  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished)
  {
    std::cout << "preprocess" << '\n';
    isFinished = false;  // TODO: secret to looping through targets i think

    typename mtFilterState::mtState& state = filterState.state_;

    // calculate common prediction
    // RvR_ = state.qRM().rotate(state.MvM() + (pred.template get<mtMeas::_gyr>() - state.gyb).cross(state.MrMR));
    const V3D MvR = state.MvM() + (meas_.aux().BwWB_ - state.gyb()).cross(MrMR_);
    RvR_ = qMR_.inverted().rotate(MvR);

    std::cout << "RvR_: " << RvR_.transpose() << '\n';
  }

  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection,
                   bool& isFinished)
  {
    std::cout << "postprocess" << '\n';
    isFinished = true;
  }
};

}  // namespace rovio

#endif /* ROVIO_DopplerUPDATE_HPP_ */
