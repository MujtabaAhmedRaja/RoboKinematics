from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel

from database import get_db
import models
from .auth import get_current_user

router = APIRouter()

class TrajectoryBase(BaseModel):
    title: str
    eq_x: str
    eq_y: str
    eq_z: str

@router.post("/")
def create_trajectory(traj: TrajectoryBase, db: Session = Depends(get_db), current_user: models.User = Depends(get_current_user)):
    db_traj = models.CustomTrajectory(
        title=traj.title,
        eq_x=traj.eq_x,
        eq_y=traj.eq_y,
        eq_z=traj.eq_z,
        owner_id=current_user.id
    )
    db.add(db_traj)
    db.commit()
    db.refresh(db_traj)
    return db_traj

@router.get("/")
def read_trajectories(db: Session = Depends(get_db), current_user: models.User = Depends(get_current_user)):
    return db.query(models.CustomTrajectory).filter(models.CustomTrajectory.owner_id == current_user.id).all()
