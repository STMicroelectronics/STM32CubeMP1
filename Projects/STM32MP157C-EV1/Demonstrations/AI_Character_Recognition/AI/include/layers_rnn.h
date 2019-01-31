#ifndef __LAYERS_RNN_H_
#define __LAYERS_RNN_H_
#pragma once

#include "layers_common.h"


AI_API_DECLARE_BEGIN

/*!
 * @typedef (*func_nl_el)
 * @ingroup layers
 * @brief Fuction pointer for generic elementwise transform
 *
 * This function pointer abstracts a generic nonlinear function applied to a
 * single element. See @ref ai_math_sqrt in @ref math_helpers as an example.
 */
typedef ai_float (*func_nl_el)(const ai_float x);

/*!
 * @struct ai_layer_lstm
 * @ingroup layers
 * @brief LSTM layer with generic nonlinearities and peephole connections
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_lstm_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE

  ai_tensor* kernel;     /**< input-to-hidden weight matrix (n_in, n_h) */
  ai_tensor* recurrent;  /**< hidden-to-hidden weight matrix (n_h, n_h) */
  ai_tensor* peephole;   /**< peephole connection weights (n_h, ) */
  ai_tensor* bias;       /**< hidden biases (n_h, ) */

  ai_size n_units;       /**< size of the hidden RNN state */

  func_nl_el activation_nl;  /**< activation nonlinearity (input to cell) */
  func_nl_el recurrent_nl;   /**< recurrent nonlinearity (hidden to cell) */
  func_nl_el out_nl;         /**< output nonlinearity (cell to hidden) */
} ai_layer_lstm;

/*!
 * @struct ai_layer_gru
 * @ingroup layers
 * @brief Gated Recurrent Unit (GRU) layer with generic nonlinearities
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_gru_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE

  ai_tensor* kernel;     /**< input-to-hidden weight matrix (n_in, n_h) */
  ai_tensor* recurrent;  /**< hidden-to-hidden weight matrix (n_h, n_h) */
  ai_tensor* bias;       /**< hidden biases (n_h, ) */

  ai_size n_units;       /**< size of the hidden RNN state */

  func_nl_el activation_nl;  /**< activation nonlinearity (input to cell) */
  func_nl_el recurrent_nl;   /**< recurrent nonlinearity (hidden to cell) */

  const ai_bool reset_after; /**< applies reset gate after matmul */
} ai_layer_gru;

/*!
 * @brief Computes the activations of a Long-Short Term Memory (LSTM) layer.
 * @ingroup layers
 *
 * Implements a Long-Short Term Layer with peephole connections:
 * \f{eqnarray*}{
 *    i_t &=& \sigma_a(x_t W_{xi} + h_{t-1} W_{hi}
 *            + w_{ci} \odot c_{t-1} + b_i)\\
 *    f_t &=& \sigma_a(x_t W_{xf} + h_{t-1} W_{hf}
 *            + w_{cf} \odot c_{t-1} + b_f)\\
 *    c_t &=& f_t \odot c_{t - 1}
 *            + i_t \odot \sigma_r(x_t W_{xc} + h_{t-1} W_{hc} + b_c)\\
 *    o_t &=& \sigma_a(x_t W_{xo} + h_{t-1} W_{ho} + w_{co} \odot c_t + b_o)\\
 *    h_t &=& o_t \odot \sigma_o(c_t)
 * \f}
 * where \f$\sigma_a\f$ is the activation nonlinearity, \f$\sigma_r\f$ is the
 * recurrent nonlinearity and \f$\sigma_o\f$ is the out nonlinearity. The
 * \f$W_x\f$, \f$W_h\f$ and \f$W_c\f$ weights are sliced from the kernel,
 * recurrent and peephole weights.
 *
 * @param layer the LSTM layer
 */
AI_INTERNAL_API
void forward_lstm(ai_layer * layer);

/*!
 * @brief Computes the activations of a Gated Recurrent Unit (GRU) layer.
 * @ingroup layers
 *
 * Implements a Gated Recurrent Unit with the formula:
 * \f{eqnarray*}{
 *    r_t &=& \sigma_a(x_t W_{xr} + h_{t - 1} W_{hr} + b_r) \\
 *    z_t &=& \sigma_a(x_t W_{xz} + h_{t - 1} W_{hz} + b_z) \\
 *    c_t &=& \sigma_r(x_t W_{xc} + r_t \odot (h_{t - 1} W_{hc} + b_{hc}) + b_c)
 *            \qquad \textnormal{when reset after is true} \\
 *    c_t &=& \sigma_r(x_t W_{xc} + (r_t \odot h_{t - 1}) W_{hc} + b_{hc} + b_c)
 *            \qquad \textnormal{when reset after is false (default)} \\
 *    h_t &=& (1 - z_t) \odot h_{t - 1} + z_t \odot c_t
 * \f}
 * where \f$\sigma_a\f$ is the activation nonlinearity and \f$\sigma_r\f$ is
 * the recurrent nonlinearity. The weights are sliced from the kernel and
 * recurrent weights.
 *
 * @param layer the GRU layer
 */
AI_INTERNAL_API
void forward_gru(ai_layer * layer);


AI_API_DECLARE_END

#endif /* __LAYERS_RNN_H_ */
