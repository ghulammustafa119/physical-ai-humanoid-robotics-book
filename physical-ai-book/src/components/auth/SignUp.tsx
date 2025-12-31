import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import styles from './styles.module.css';

interface SignUpFormData {
  email: string;
  password: string;
  confirmPassword: string;
  programmingLevel: string;
  pythonLevel: string;
  aiMlLevel: string;
  roboticsLevel: string;
  systemType: string;
  gpuAvailability: string;
  hardwareAccess: string;
  simulatorExperience: string[];
}

const SignUp: React.FC = () => {
  const { signUp } = useAuth();
  const [formData, setFormData] = useState<SignUpFormData>({
    email: '',
    password: '',
    confirmPassword: '',
    programmingLevel: '',
    pythonLevel: '',
    aiMlLevel: '',
    roboticsLevel: '',
    systemType: '',
    gpuAvailability: '',
    hardwareAccess: '',
    simulatorExperience: [],
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showProfileQuestions, setShowProfileQuestions] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSimulatorChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { value, checked } = e.target;
    setFormData(prev => {
      const simulators = prev.simulatorExperience || [];
      if (checked) {
        return {
          ...prev,
          simulatorExperience: [...simulators, value]
        };
      } else {
        return {
          ...prev,
          simulatorExperience: simulators.filter(s => s !== value)
        };
      }
    });
  };

  const validateForm = () => {
    if (!formData.email || !formData.password) {
      setError('Email and password are required');
      return false;
    }
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return false;
    }
    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }
    return true;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (!validateForm()) {
      return;
    }

    setLoading(true);

    try {
      const profileData = showProfileQuestions ? {
        programming_level: formData.programmingLevel,
        python_level: formData.pythonLevel,
        ai_ml_level: formData.aiMlLevel,
        robotics_level: formData.roboticsLevel,
        system_type: formData.systemType,
        gpu_availability: formData.gpuAvailability,
        hardware_access: formData.hardwareAccess,
        simulator_experience: formData.simulatorExperience,
      } : undefined;

      const success = await signUp(formData.email, formData.password, profileData);

      if (!success) {
        setError('Signup failed. Please try again.');
      }
    } catch (err: any) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <h2>Sign Up</h2>
      {error && <div className={styles.error}>{error}</div>}

      <form onSubmit={handleSubmit} className={styles.authForm}>
        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
            className={styles.input}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
            className={styles.input}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="confirmPassword">Confirm Password</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            required
            className={styles.input}
          />
        </div>

        <div className={styles.formGroup}>
          <button
            type="button"
            onClick={() => setShowProfileQuestions(!showProfileQuestions)}
            className={styles.toggleButton}
          >
            {showProfileQuestions ? 'Hide Profile Questions' : 'Add Profile Information (Optional)'}
          </button>
        </div>

        {showProfileQuestions && (
          <div className={styles.profileSection}>
            <h3>Profile Information</h3>

            <div className={styles.formGroup}>
              <label htmlFor="programmingLevel">Programming Experience</label>
              <select
                id="programmingLevel"
                name="programmingLevel"
                value={formData.programmingLevel}
                onChange={handleChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="pythonLevel">Python Experience</label>
              <select
                id="pythonLevel"
                name="pythonLevel"
                value={formData.pythonLevel}
                onChange={handleChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="none">None</option>
                <option value="basic">Basic</option>
                <option value="strong">Strong</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="aiMlLevel">AI/ML Experience</label>
              <select
                id="aiMlLevel"
                name="aiMlLevel"
                value={formData.aiMlLevel}
                onChange={handleChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="none">None</option>
                <option value="basic">Basic</option>
                <option value="applied">Applied</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="roboticsLevel">Robotics Experience</label>
              <select
                id="roboticsLevel"
                name="roboticsLevel"
                value={formData.roboticsLevel}
                onChange={handleChange}
                className={styles.input}
              >
                <option value="">Select level</option>
                <option value="none">None</option>
                <option value="academic">Academic</option>
                <option value="practical">Practical</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="systemType">System Type</label>
              <select
                id="systemType"
                name="systemType"
                value={formData.systemType}
                onChange={handleChange}
                className={styles.input}
              >
                <option value="">Select system</option>
                <option value="laptop">Laptop</option>
                <option value="desktop">Desktop</option>
                <option value="cloud">Cloud</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="gpuAvailability">GPU Availability</label>
              <select
                id="gpuAvailability"
                name="gpuAvailability"
                value={formData.gpuAvailability}
                onChange={handleChange}
                className={styles.input}
              >
                <option value="">Select GPU</option>
                <option value="none">None</option>
                <option value="integrated">Integrated</option>
                <option value="nvidia_cuda">NVIDIA CUDA GPU</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="hardwareAccess">Hardware Access</label>
              <select
                id="hardwareAccess"
                name="hardwareAccess"
                value={formData.hardwareAccess}
                onChange={handleChange}
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
                    checked={formData.simulatorExperience.includes('gazebo')}
                    onChange={handleSimulatorChange}
                  />
                  Gazebo
                </label>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    value="isaac_sim"
                    checked={formData.simulatorExperience.includes('isaac_sim')}
                    onChange={handleSimulatorChange}
                  />
                  Isaac Sim
                </label>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    value="unity"
                    checked={formData.simulatorExperience.includes('unity')}
                    onChange={handleSimulatorChange}
                  />
                  Unity
                </label>
              </div>
            </div>
          </div>
        )}

        <button type="submit" disabled={loading} className={styles.button}>
          {loading ? 'Signing Up...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignUp;