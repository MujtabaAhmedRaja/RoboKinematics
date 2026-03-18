from sqlalchemy import Column, Integer, String, JSON, ForeignKey
from sqlalchemy.orm import relationship
from database import Base

class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    username = Column(String, unique=True, index=True)
    hashed_password = Column(String)

    configs = relationship("RobotConfig", back_populates="owner")
    trajectories = relationship("CustomTrajectory", back_populates="owner")

class RobotConfig(Base):
    __tablename__ = "robot_configs"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, index=True)
    dof = Column(Integer)
    dh_params = Column(JSON)
    locked_joints = Column(JSON)
    owner_id = Column(Integer, ForeignKey("users.id"))

    owner = relationship("User", back_populates="configs")

class CustomTrajectory(Base):
    __tablename__ = "custom_trajectories"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, index=True)
    eq_x = Column(String)
    eq_y = Column(String)
    eq_z = Column(String)
    owner_id = Column(Integer, ForeignKey("users.id"))

    owner = relationship("User", back_populates="trajectories")
