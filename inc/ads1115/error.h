/*
 * @Author: Lukasz
 * @Date:   22-10-2018
 * @Last Modified by:   Lukasz
 * @Last Modified time: 22-10-2018
 */

#pragma once
namespace ADS1115
{

enum class Error
{
  NONE,
  ERR,
  BUSY,
  TIMEOUT,
  BAD_ADDRESS,
  IO,
  INVALID_ARG,
};
};
