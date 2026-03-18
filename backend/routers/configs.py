from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List

from database import get_db
import models
from .auth import get_current_user

router = APIRouter()

class ConfigBase(BaseModel):
    title: str
    dof: int
    dh_params: list
    locked_joints: list

@router.post("/")
def create_config(config: ConfigBase, db: Session = Depends(get_db), current_user: models.User = Depends(get_current_user)):
    db_config = models.RobotConfig(
        title=config.title,
        dof=config.dof,
        dh_params=config.dh_params,
        locked_joints=config.locked_joints,
        owner_id=current_user.id
    )
    db.add(db_config)
    db.commit()
    db.refresh(db_config)
    return db_config

@router.get("/")
def read_configs(db: Session = Depends(get_db), current_user: models.User = Depends(get_current_user)):
    configs = db.query(models.RobotConfig).filter(models.RobotConfig.owner_id == current_user.id).all()
    return configs
