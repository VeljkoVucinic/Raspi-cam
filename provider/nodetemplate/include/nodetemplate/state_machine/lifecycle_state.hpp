//! Elternklasse der States
#pragma once

#include <cstdio>
#include <chrono>
#include <string>
#include <functional>

class LifecycleState
{
public:
    LifecycleState();
    ~LifecycleState();

    virtual std::string GetName();

    virtual bool OnBegin();
    virtual bool OnTransition(std::string);
    virtual void OnError();

    virtual void OnUpdate();

};