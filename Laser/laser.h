/**
  ******************************************************************************
  * @file    laser.h
  * @brief   激光控制接口（与振镜无关，独立模块）
  ******************************************************************************
  */
#ifndef __LASER_H__
#define __LASER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief 初始化激光控制所依赖的硬件
  */
void Laser_Init(void);

/**
  * @brief 打开激光
  */
void Laser_On(void);

/**
  * @brief 关闭激光
  */
void Laser_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* __LASER_H__ */
