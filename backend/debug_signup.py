import sys
import os
from sqlmodel import Session, create_engine, select

# Add parent directory to sys.path
sys.path.append(os.getcwd())

from src.db.connection import engine
from src.services.auth_service import AuthService
from src.models.user import UserCreate, User

def test_signup_debug():
    # Clean up existing user if any
    with Session(engine) as session:
        existing = session.exec(select(User).where(User.email == "ghulammustafabhutto77@gmail.com")).first()
        if existing:
            print(f"User already exists. ID: {existing.id}")
            # Optional: session.delete(existing); session.commit()

    # Try registration
    try:
        user_in = UserCreate(
            email="ghulammustafabhutto77@gmail.com",
            password="Password123" # Valid password
        )

        with Session(engine) as session:
            auth_service = AuthService(session)
            print("Attempting to create user...")
            user, token = auth_service.create_user(user_in)
            print(f"SUCCESS: User created with ID {user.id}")
            print(f"Session Token: {token}")

    except Exception as e:
        print(f"FAILURE: {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_signup_debug()
