-- Create authentication tables for user management and session tracking

-- Create users table (managed by Better Auth)
CREATE TABLE IF NOT EXISTS "user" (
    "id" TEXT PRIMARY KEY,
    "email" TEXT NOT NULL UNIQUE,
    "password_hash" TEXT NOT NULL,
    "created_at" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    "updated_at" TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Create sessions table (managed by Better Auth)
CREATE TABLE IF NOT EXISTS "session" (
    "id" TEXT PRIMARY KEY,
    "user_id" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
    "token" TEXT NOT NULL UNIQUE,
    "expires_at" TIMESTAMP WITH TIME ZONE NOT NULL,
    "created_at" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    "ip_address" TEXT,
    "user_agent" TEXT,
    CONSTRAINT fk_user FOREIGN KEY ("user_id") REFERENCES "user"("id")
);

-- Create user profiles table (custom for personalization)
CREATE TABLE IF NOT EXISTS "user_profile" (
    "id" TEXT PRIMARY KEY,
    "user_id" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
    "programming_level" TEXT CHECK ("programming_level" IN ('beginner', 'intermediate', 'advanced')),
    "python_level" TEXT CHECK ("python_level" IN ('none', 'basic', 'strong')),
    "ai_ml_level" TEXT CHECK ("ai_ml_level" IN ('none', 'basic', 'applied')),
    "robotics_level" TEXT CHECK ("robotics_level" IN ('none', 'academic', 'practical')),
    "system_type" TEXT CHECK ("system_type" IN ('laptop', 'desktop', 'cloud')),
    "gpu_availability" TEXT CHECK ("gpu_availability" IN ('none', 'integrated', 'nvidia_cuda')),
    "hardware_access" TEXT CHECK ("hardware_access" IN ('none', 'simulators', 'real')),
    "simulator_experience" TEXT[],
    "profile_completeness" REAL DEFAULT 0.0,
    "created_at" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    "updated_at" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    CONSTRAINT fk_user FOREIGN KEY ("user_id") REFERENCES "user"("id")
);

-- Create indexes for efficient queries
CREATE INDEX IF NOT EXISTS idx_user_email ON "user"("email");
CREATE INDEX IF NOT EXISTS idx_session_user_id ON "session"("user_id");
CREATE INDEX IF NOT EXISTS idx_session_token ON "session"("token");
CREATE INDEX IF NOT EXISTS idx_session_expires ON "session"("expires_at");
CREATE INDEX IF NOT EXISTS idx_user_profile_user_id ON "user_profile"("user_id");
CREATE INDEX IF NOT EXISTS idx_user_profile_completeness ON "user_profile"("profile_completeness");

-- Insert sample data for testing (optional)
-- INSERT INTO "user" (id, email, password_hash) VALUES
-- ('usr_test_001', 'test@example.com', '$2b$12$hashed_password_here');

-- INSERT INTO "user_profile" (id, user_id, programming_level, python_level, ai_ml_level, robotics_level, system_type, gpu_availability, hardware_access, simulator_experience, profile_completeness) VALUES
-- ('prof_test_001', 'usr_test_001', 'intermediate', 'strong', 'basic', 'none', 'laptop', 'integrated', 'simulators', ARRAY['gazebo'], 0.875);

-- Function to calculate profile completeness
CREATE OR REPLACE FUNCTION calculate_profile_completeness(
    programming_level TEXT,
    python_level TEXT,
    ai_ml_level TEXT,
    robotics_level TEXT,
    system_type TEXT,
    gpu_availability TEXT,
    hardware_access TEXT,
    simulator_experience TEXT[]
) RETURNS REAL AS $$
DECLARE
    total_fields INTEGER := 8;
    filled_fields INTEGER := 0;
BEGIN
    IF programming_level IS NOT NULL THEN filled_fields := filled_fields + 1; END IF;
    IF python_level IS NOT NULL THEN filled_fields := filled_fields + 1; END IF;
    IF ai_ml_level IS NOT NULL THEN filled_fields := filled_fields + 1; END IF;
    IF robotics_level IS NOT NULL THEN filled_fields := filled_fields + 1; END IF;
    IF system_type IS NOT NULL THEN filled_fields := filled_fields + 1; END IF;
    IF gpu_availability IS NOT NULL THEN filled_fields := filled_fields + 1; END IF;
    IF hardware_access IS NOT NULL THEN filled_fields := filled_fields + 1; END IF;
    IF simulator_experience IS NOT NULL AND array_length(simulator_experience, 1) > 0 THEN filled_fields := filled_fields + 1; END IF;

    RETURN ROUND((filled_fields::REAL / total_fields::REAL) * 100) / 100;
END;
$$ LANGUAGE plpgsql;

-- Update profile_completeness when user_profile is updated
CREATE OR REPLACE FUNCTION update_profile_completeness()
RETURNS TRIGGER AS $$
BEGIN
    NEW.profile_completeness := calculate_profile_completeness(
        NEW.programming_level,
        NEW.python_level,
        NEW.ai_ml_level,
        NEW.robotics_level,
        NEW.system_type,
        NEW.gpu_availability,
        NEW.hardware_access,
        NEW.simulator_experience
    );
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Create trigger to update profile_completeness
CREATE TRIGGER trigger_update_profile_completeness
    BEFORE INSERT OR UPDATE ON user_profile
    FOR EACH ROW
    EXECUTE FUNCTION update_profile_completeness();

-- ROLLBACK command (for testing - uncomment to revert)
-- ROLLBACK;

-- To revert all changes:
-- DROP TABLE IF EXISTS user_profile, session, "user";