import React, { useState, useEffect } from 'react';
import { useAuth } from './AuthProvider';
import { updateUserProfile } from '../../utils/auth';
import styles from './styles.module.css';

interface ProfileData {
  id: string;
  user_id: string;
  programming_level: string;
  python_level: string;
  ai_ml_level: string;
  robotics_level: string;
  system_type: string;
  gpu_availability: string;
  hardware_access: string;
  simulator_experience: string[];
  profile_completeness: number;
  created_at: string;
  updated_at: string;
}

const Profile: React.FC = () => {
  const { user, loading: authLoading, refreshProfile } = useAuth();
  const [profile, setProfile] = useState<ProfileData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [editData, setEditData] = useState<Partial<ProfileData>>({});
  const [editLoading, setEditLoading] = useState(false);

  useEffect(() => {
    // If auth is done loading, update profile and stop loading screen
    if (!authLoading) {
      if (user) {
        setProfile(user.profile || null);
        setError(null);
      } else {
        setError('Not authenticated. Please sign in.');
      }
      setLoading(false);
    }
  }, [user, authLoading]);

  // Profile is loaded from context, no need to fetch separately
  // Use refreshProfile function when we need to reload profile data

  const handleEditToggle = () => {
    setIsEditing(!isEditing);
    if (!isEditing && user?.profile) {
      // Initialize edit data with current profile
      setEditData({
        programming_level: user.profile.programming_level,
        python_level: user.profile.python_level,
        ai_ml_level: user.profile.ai_ml_level,
        robotics_level: user.profile.robotics_level,
        system_type: user.profile.system_type,
        gpu_availability: user.profile.gpu_availability,
        hardware_access: user.profile.hardware_access,
        simulator_experience: user.profile.simulator_experience,
      });
    }
  };

  const handleEditChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setEditData(prev => ({ ...prev, [name]: value }));
  };

  const handleSimulatorChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { value, checked } = e.target;
    setEditData(prev => {
      const simulators = prev.simulator_experience || [];
      if (checked) {
        return {
          ...prev,
          simulator_experience: [...simulators, value]
        };
      } else {
        return {
          ...prev,
          simulator_experience: simulators.filter(s => s !== value)
        };
      }
    });
  };

  const handleSave = async () => {
    if (!user) return;

    setEditLoading(true);
    try {
      // Use the updateUserProfile utility function from auth utils
      const result = await updateUserProfile(editData);
      setProfile(result);
      setIsEditing(false);

      // Refresh the profile in context
      await refreshProfile();
    } catch (err: any) {
      setError(err.message || 'Failed to update profile');
    } finally {
      setEditLoading(false);
    }
  };

  const handleCancel = () => {
    setIsEditing(false);
    setEditData({});
  };

  const handleLogout = async () => {
    try {
      await signOut(); // From useAuth context
      window.location.href = '/physical-ai-humanoid-robotics-book/';
    } catch (err) {
      console.error('Logout failed:', err);
    }
  };

  if (loading) {
    return <div className={styles.loading}>Loading profile...</div>;
  }

  if (error) {
    return <div className={styles.error}>{error}</div>;
  }

  if (!profile) {
    return <div className={styles.error}>No profile data found</div>;
  }

  return (
    <div className={styles.profileContainer}>
      <h2>User Profile</h2>

      <div className={styles.profileInfo}>
        <div className={styles.infoItem}>
          <strong>Email:</strong> {user?.email}
        </div>
        <div className={styles.infoItem}>
          <strong>Profile Completeness:</strong> {Math.round(profile.profile_completeness * 100)}%
        </div>
      </div>

      <div className={styles.profileDetails}>
        <h3>Background Information</h3>

        {isEditing ? (
          <div className={styles.editForm}>
            <div className={styles.formGroup}>
              <label htmlFor="programming_level">Programming Experience</label>
              <select
                id="programming_level"
                name="programming_level"
                value={editData.programming_level || ''}
                onChange={handleEditChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="python_level">Python Experience</label>
              <select
                id="python_level"
                name="python_level"
                value={editData.python_level || ''}
                onChange={handleEditChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="none">None</option>
                <option value="basic">Basic</option>
                <option value="strong">Strong</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="ai_ml_level">AI/ML Experience</label>
              <select
                id="ai_ml_level"
                name="ai_ml_level"
                value={editData.ai_ml_level || ''}
                onChange={handleEditChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="none">None</option>
                <option value="basic">Basic</option>
                <option value="applied">Applied</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="robotics_level">Robotics Experience</label>
              <select
                id="robotics_level"
                name="robotics_level"
                value={editData.robotics_level || ''}
                onChange={handleEditChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="none">None</option>
                <option value="academic">Academic</option>
                <option value="practical">Practical</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="system_type">System Type</label>
              <select
                id="system_type"
                name="system_type"
                value={editData.system_type || ''}
                onChange={handleEditChange}
                className={styles.input}
              >
                <option value="">Select system</option>
                <option value="laptop">Laptop</option>
                <option value="desktop">Desktop</option>
                <option value="cloud">Cloud</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="gpu_availability">GPU Availability</label>
              <select
                id="gpu_availability"
                name="gpu_availability"
                value={editData.gpu_availability || ''}
                onChange={handleEditChange}
                className={styles.input}
              >
                <option value="">Select GPU</option>
                <option value="none">None</option>
                <option value="integrated">Integrated</option>
                <option value="nvidia_cuda">NVIDIA CUDA GPU</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="hardware_access">Hardware Access</label>
              <select
                id="hardware_access"
                name="hardware_access"
                value={editData.hardware_access || ''}
                onChange={handleEditChange}
                className={styles.input}
              >
                <option value="">Select access</option>
                <option value="none">None</option>
                <option value="simulators">Simulators only</option>
                <option value="real">Real robot</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label>Simulator Experience</label>
              <div className={styles.checkboxGroup}>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    value="gazebo"
                    checked={editData.simulator_experience?.includes('gazebo') || false}
                    onChange={handleSimulatorChange}
                  />
                  Gazebo
                </label>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    value="isaac_sim"
                    checked={editData.simulator_experience?.includes('isaac_sim') || false}
                    onChange={handleSimulatorChange}
                  />
                  Isaac Sim
                </label>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    value="unity"
                    checked={editData.simulator_experience?.includes('unity') || false}
                    onChange={handleSimulatorChange}
                  />
                  Unity
                </label>
              </div>
            </div>

            <div className={styles.buttonGroup}>
              <button
                onClick={handleSave}
                disabled={editLoading}
                className={styles.button}
              >
                {editLoading ? 'Saving...' : 'Save'}
              </button>
              <button
                onClick={handleCancel}
                className={styles.cancelButton}
              >
                Cancel
              </button>
            </div>
          </div>
        ) : (
          <div className={styles.profileView}>
            <div className={styles.infoItem}>
              <strong>Programming Level:</strong> {profile.programming_level || 'Not specified'}
            </div>
            <div className={styles.infoItem}>
              <strong>Python Level:</strong> {profile.python_level || 'Not specified'}
            </div>
            <div className={styles.infoItem}>
              <strong>AI/ML Level:</strong> {profile.ai_ml_level || 'Not specified'}
            </div>
            <div className={styles.infoItem}>
              <strong>Robotics Level:</strong> {profile.robotics_level || 'Not specified'}
            </div>
            <div className={styles.infoItem}>
              <strong>System Type:</strong> {profile.system_type || 'Not specified'}
            </div>
            <div className={styles.infoItem}>
              <strong>GPU Availability:</strong> {profile.gpu_availability || 'Not specified'}
            </div>
            <div className={styles.infoItem}>
              <strong>Hardware Access:</strong> {profile.hardware_access || 'Not specified'}
            </div>
            <div className={styles.infoItem}>
              <strong>Simulator Experience:</strong> {profile.simulator_experience?.join(', ') || 'None'}
            </div>

            <button
              onClick={handleEditToggle}
              className={styles.editButton}
            >
              Edit Profile
            </button>
          </div>
        )}
      </div>
    </div>
  );
};

export default Profile;